// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG10/20 system console
 *
 * Text output goes straight into the GRAM of the R61524 LCD controller
 * (no framebuffer in RAM: with 2 MiB of memory the 162 KiB VRAM would
 * be a luxury), input comes from polling the SH7305 key scan unit.
 * A small VT100 subset is interpreted so that BusyBox ash line editing
 * and vi work. The bottom text row is a status line that shows the
 * keyboard modifier state.
 */
#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <asm/barrier.h>
#include <mach/fxcg.h>

#include "font6x8.h"

/* R61524: 16-bit interface, register select on port R bit 4 */
#define LCD_DATA	((void __iomem *)0xb4000000)
#define PRDR		((void __iomem *)0xa405013c)
#define LCD_RS		0x10

#define R_HADDR		0x200
#define R_VADDR		0x201
#define R_GRAM		0x202
#define R_HSA		0x210
#define R_HEA		0x211
#define R_VSA		0x212
#define R_VEA		0x213

/* The Casio OS's 384x216 area sits at panel H = 389 - x, V = y */
#define LCD_ORIGIN_H	389
#define LCD_PANEL_WIDTH		396
#define LCD_PANEL_HEIGHT	224

/* SH7305 key scan unit: six 16-bit words, word = col/2, bit = row + 8*(col&1) */
#define KEYSC_DATA	((void __iomem *)0xa44b0000)

#define FONT_W		6
#define FONT_H		8
#define COLS		(FXCG_LCD_WIDTH / FONT_W)	/* 64 */
#define ROWS		(FXCG_LCD_HEIGHT / FONT_H)	/* 27 */
#define TROWS		(ROWS - 1)			/* terminal rows */
#define STATUS_ROW	(ROWS - 1)

#define ATTR(fg, bg)	((fg) | ((bg) << 4))
/* dark text on white, like the Casio OS and the panel's white border */
#define DEF_FG		0
#define DEF_BG		15
#define DEF_ATTR	ATTR(DEF_FG, DEF_BG)
#define STATUS_ATTR	ATTR(0, 7)	/* black on light grey */

struct cell {
	u8 ch;
	u8 attr;
};

static const u16 palette[16] = {
	0x0000, 0xb000, 0x0580, 0xb580, 0x0016, 0xb016, 0x0596, 0xc618,
	0x6b4d, 0xf800, 0x07e0, 0xffe0, 0x435f, 0xf81f, 0x07ff, 0xffff,
};

static DEFINE_SPINLOCK(con_lock);

static struct cell screen[ROWS][COLS];	/* wanted contents */
static struct cell shown[ROWS][COLS];	/* what the LCD shows */

static struct {
	int x, y;
	int saved_x, saved_y;
	u8 attr, saved_attr;
	bool reverse, bold;
	bool wrap_pending;
	bool cursor_on;
	int shown_x, shown_y;	/* where the cursor is drawn, -1 = nowhere */
	/* escape sequence parser */
	enum { ST_NORMAL, ST_ESC, ST_CSI } state;
	bool csi_private;
	int npar;
	int par[8];
} vt;

static struct tty_port fxcg_port;
static struct tty_driver *fxcg_tty_driver;

/* ------------------------------------------------------------------ LCD */

static inline void lcd_select(u16 reg)
{
	__raw_writeb(__raw_readb(PRDR) & ~LCD_RS, PRDR);
	wmb();
	__raw_writew(reg, LCD_DATA);
	wmb();
	__raw_writeb(__raw_readb(PRDR) | LCD_RS, PRDR);
	wmb();
}

static inline void lcd_reg(u16 reg, u16 val)
{
	lcd_select(reg);
	__raw_writew(val, LCD_DATA);
}

/* Open a window on [x0,x1]x[y0,y1] and select GRAM for streaming. */
static void lcd_window(int x0, int y0, int x1, int y1)
{
	lcd_reg(R_HSA, LCD_ORIGIN_H - x1);
	lcd_reg(R_HEA, LCD_ORIGIN_H - x0);
	lcd_reg(R_VSA, y0);
	lcd_reg(R_VEA, y1);
	lcd_reg(R_HADDR, 0);
	lcd_reg(R_VADDR, 0);
	lcd_select(R_GRAM);
}

static void lcd_draw_cell(int row, int col, struct cell c, bool cursor)
{
	const u8 *glyph;
	u16 fg = palette[c.attr & 15], bg = palette[c.attr >> 4];
	int y, x;

	if (c.ch < FXCG_FONT_FIRST || c.ch > FXCG_FONT_LAST)
		c.ch = ' ';
	glyph = fxcg_font[c.ch - FXCG_FONT_FIRST];

	lcd_window(col * FONT_W, row * FONT_H,
		   col * FONT_W + FONT_W - 1, row * FONT_H + FONT_H - 1);
	for (y = 0; y < FONT_H; y++) {
		u8 bits = glyph[y];

		if (cursor && y >= FONT_H - 2)
			bits = 0xfc;
		for (x = 0; x < FONT_W; x++)
			__raw_writew(bits & (0x80 >> x) ? fg : bg, LCD_DATA);
	}
}

/* Fill the whole panel, including the border around the Casio OS area. */
static void lcd_fill(u16 color)
{
	int i;

	lcd_reg(R_HSA, 0);
	lcd_reg(R_HEA, LCD_PANEL_WIDTH - 1);
	lcd_reg(R_VSA, 0);
	lcd_reg(R_VEA, LCD_PANEL_HEIGHT - 1);
	lcd_reg(R_HADDR, 0);
	lcd_reg(R_VADDR, 0);
	lcd_select(R_GRAM);
	for (i = 0; i < LCD_PANEL_WIDTH * LCD_PANEL_HEIGHT; i++)
		__raw_writew(color, LCD_DATA);
}

/* Push every changed cell (and the cursor) to the LCD. */
static void con_flush(void)
{
	bool cursor_moved = vt.shown_x != vt.x || vt.shown_y != vt.y ||
			    !vt.cursor_on;
	int r, c;

	if (cursor_moved && vt.shown_y >= 0) {
		lcd_draw_cell(vt.shown_y, vt.shown_x,
			      screen[vt.shown_y][vt.shown_x], false);
		shown[vt.shown_y][vt.shown_x] = screen[vt.shown_y][vt.shown_x];
		vt.shown_y = -1;
	}

	for (r = 0; r < ROWS; r++)
		for (c = 0; c < COLS; c++) {
			if (screen[r][c].ch == shown[r][c].ch &&
			    screen[r][c].attr == shown[r][c].attr &&
			    !(r == vt.y && c == vt.x && vt.shown_y < 0))
				continue;
			lcd_draw_cell(r, c, screen[r][c],
				      vt.cursor_on && r == vt.y && c == vt.x);
			shown[r][c] = screen[r][c];
		}

	if (vt.cursor_on && vt.shown_y < 0) {
		lcd_draw_cell(vt.y, vt.x, screen[vt.y][vt.x], true);
		vt.shown_x = vt.x;
		vt.shown_y = vt.y;
	}
}

/* ------------------------------------------------------------ terminal */

static u8 cur_attr(void)
{
	u8 fg = vt.attr & 15, bg = vt.attr >> 4;

	if (vt.bold && fg < 8)
		fg += 8;
	return vt.reverse ? ATTR(bg, fg) : ATTR(fg, bg);
}

static void clear_cells(int row, int from, int to)
{
	int c;

	for (c = from; c < to; c++) {
		screen[row][c].ch = ' ';
		screen[row][c].attr = ATTR(DEF_FG, vt.attr >> 4);
	}
}

static void scroll_up(int top, int bottom, int n)
{
	int r;

	if (n > bottom - top)
		n = bottom - top;
	for (r = top; r < bottom - n; r++)
		memcpy(screen[r], screen[r + n], sizeof(screen[r]));
	for (; r < bottom; r++)
		clear_cells(r, 0, COLS);
}

static void scroll_down(int top, int bottom, int n)
{
	int r;

	if (n > bottom - top)
		n = bottom - top;
	for (r = bottom - 1; r >= top + n; r--)
		memcpy(screen[r], screen[r - n], sizeof(screen[r]));
	for (; r >= top; r--)
		clear_cells(r, 0, COLS);
}

static void newline(void)
{
	if (++vt.y >= TROWS) {
		vt.y = TROWS - 1;
		scroll_up(0, TROWS, 1);
	}
}

static void put_char(u8 ch)
{
	if (vt.wrap_pending) {
		vt.wrap_pending = false;
		vt.x = 0;
		newline();
	}
	screen[vt.y][vt.x].ch = ch;
	screen[vt.y][vt.x].attr = cur_attr();
	if (vt.x == COLS - 1)
		vt.wrap_pending = true;
	else
		vt.x++;
}

static void reset_attrs(void)
{
	vt.attr = DEF_ATTR;
	vt.reverse = vt.bold = false;
}

static void set_cursor(int x, int y)
{
	vt.x = clamp(x, 0, COLS - 1);
	vt.y = clamp(y, 0, TROWS - 1);
	vt.wrap_pending = false;
}

static void respond(const char *s)
{
	tty_insert_flip_string(&fxcg_port, s, strlen(s));
	tty_flip_buffer_push(&fxcg_port);
}

static void do_sgr(void)
{
	int i;

	if (vt.npar == 0)
		vt.par[vt.npar++] = 0;
	for (i = 0; i < vt.npar; i++) {
		int p = vt.par[i];

		if (p == 0)
			reset_attrs();
		else if (p == 1)
			vt.bold = true;
		else if (p == 22)
			vt.bold = false;
		else if (p == 7)
			vt.reverse = true;
		else if (p == 27)
			vt.reverse = false;
		else if (p >= 30 && p <= 37)
			vt.attr = (vt.attr & 0xf0) | (p - 30);
		else if (p == 39)
			vt.attr = (vt.attr & 0xf0) | DEF_FG;
		else if (p >= 40 && p <= 47)
			vt.attr = (vt.attr & 0x0f) | ((p - 40) << 4);
		else if (p == 49)
			vt.attr = (vt.attr & 0x0f) | (DEF_BG << 4);
		else if (p >= 90 && p <= 97)
			vt.attr = (vt.attr & 0xf0) | (p - 90 + 8);
	}
}

static void do_csi(u8 cmd)
{
	int p0 = vt.npar > 0 ? vt.par[0] : 0;
	int n = p0 ? p0 : 1;
	char buf[24];

	if (vt.csi_private) {
		if ((cmd == 'h' || cmd == 'l') && p0 == 25)
			vt.cursor_on = cmd == 'h';
		return;
	}

	switch (cmd) {
	case 'A':
		set_cursor(vt.x, vt.y - n);
		break;
	case 'B':
	case 'e':
		set_cursor(vt.x, vt.y + n);
		break;
	case 'C':
	case 'a':
		set_cursor(vt.x + n, vt.y);
		break;
	case 'D':
		set_cursor(vt.x - n, vt.y);
		break;
	case 'E':
		set_cursor(0, vt.y + n);
		break;
	case 'F':
		set_cursor(0, vt.y - n);
		break;
	case 'G':
	case '`':
		set_cursor(n - 1, vt.y);
		break;
	case 'd':
		set_cursor(vt.x, n - 1);
		break;
	case 'H':
	case 'f':
		set_cursor((vt.npar > 1 && vt.par[1] ? vt.par[1] : 1) - 1, n - 1);
		break;
	case 'J':
		if (p0 == 0) {
			clear_cells(vt.y, vt.x, COLS);
			for (n = vt.y + 1; n < TROWS; n++)
				clear_cells(n, 0, COLS);
		} else if (p0 == 1) {
			clear_cells(vt.y, 0, vt.x + 1);
			for (n = 0; n < vt.y; n++)
				clear_cells(n, 0, COLS);
		} else {
			for (n = 0; n < TROWS; n++)
				clear_cells(n, 0, COLS);
		}
		break;
	case 'K':
		if (p0 == 0)
			clear_cells(vt.y, vt.x, COLS);
		else if (p0 == 1)
			clear_cells(vt.y, 0, vt.x + 1);
		else
			clear_cells(vt.y, 0, COLS);
		break;
	case 'L':
		scroll_down(vt.y, TROWS, n);
		break;
	case 'M':
		scroll_up(vt.y, TROWS, n);
		break;
	case 'P':
		n = min(n, COLS - vt.x);
		memmove(&screen[vt.y][vt.x], &screen[vt.y][vt.x + n],
			(COLS - vt.x - n) * sizeof(struct cell));
		clear_cells(vt.y, COLS - n, COLS);
		break;
	case '@':
		n = min(n, COLS - vt.x);
		memmove(&screen[vt.y][vt.x + n], &screen[vt.y][vt.x],
			(COLS - vt.x - n) * sizeof(struct cell));
		clear_cells(vt.y, vt.x, vt.x + n);
		break;
	case 'X':
		clear_cells(vt.y, vt.x, min(vt.x + n, COLS));
		break;
	case 'S':
		scroll_up(0, TROWS, n);
		break;
	case 'T':
		scroll_down(0, TROWS, n);
		break;
	case 'm':
		do_sgr();
		break;
	case 'n':
		if (p0 == 6) {
			snprintf(buf, sizeof(buf), "\033[%d;%dR",
				 vt.y + 1, vt.x + 1);
			respond(buf);
		} else if (p0 == 5) {
			respond("\033[0n");
		}
		break;
	case 's':
		vt.saved_x = vt.x;
		vt.saved_y = vt.y;
		break;
	case 'u':
		set_cursor(vt.saved_x, vt.saved_y);
		break;
	}
}

static void vt_reset(void)
{
	int r;

	reset_attrs();
	for (r = 0; r < TROWS; r++)
		clear_cells(r, 0, COLS);
	vt.x = vt.y = 0;
	vt.wrap_pending = false;
	vt.cursor_on = true;
	vt.state = ST_NORMAL;
}

static void vt_putc(u8 ch)
{
	switch (vt.state) {
	case ST_ESC:
		vt.state = ST_NORMAL;
		switch (ch) {
		case '[':
			vt.state = ST_CSI;
			vt.csi_private = false;
			vt.npar = 0;
			memset(vt.par, 0, sizeof(vt.par));
			break;
		case '7':
			vt.saved_x = vt.x;
			vt.saved_y = vt.y;
			vt.saved_attr = vt.attr;
			break;
		case '8':
			set_cursor(vt.saved_x, vt.saved_y);
			vt.attr = vt.saved_attr;
			break;
		case 'D':
			newline();
			break;
		case 'E':
			vt.x = 0;
			newline();
			break;
		case 'M':
			if (vt.y == 0)
				scroll_down(0, TROWS, 1);
			else
				vt.y--;
			break;
		case 'c':
			vt_reset();
			break;
		}
		return;
	case ST_CSI:
		if (ch >= '0' && ch <= '9') {
			if (vt.npar == 0)
				vt.npar = 1;
			vt.par[vt.npar - 1] = min(vt.par[vt.npar - 1] * 10 +
						  (ch - '0'), 9999);
		} else if (ch == ';') {
			if (vt.npar == 0)
				vt.npar = 1;
			if (vt.npar < ARRAY_SIZE(vt.par))
				vt.npar++;
		} else if (ch == '?' || ch == '>') {
			vt.csi_private = true;
		} else if (ch >= 0x40 && ch <= 0x7e) {
			vt.state = ST_NORMAL;
			do_csi(ch);
		} else if (ch < 0x20) {
			vt.state = ST_NORMAL;	/* abort on control chars */
		}
		return;
	case ST_NORMAL:
		break;
	}

	switch (ch) {
	case '\033':
		vt.state = ST_ESC;
		break;
	case '\r':
		vt.x = 0;
		vt.wrap_pending = false;
		break;
	case '\n':
	case '\v':
	case '\f':
		vt.wrap_pending = false;
		newline();
		break;
	case '\b':
		if (vt.x > 0)
			vt.x--;
		vt.wrap_pending = false;
		break;
	case '\t':
		do {
			put_char(' ');
		} while (vt.x % 8 && !vt.wrap_pending);
		break;
	case '\a':
	case 0:
		break;
	default:
		if (ch >= 0x20)
			put_char(ch);
		break;
	}
}

static void con_write_locked(const u8 *s, size_t n)
{
	while (n--)
		vt_putc(*s++);
	con_flush();
}

/* ------------------------------------------------------------- console */

static void fxcg_console_write(struct console *con, const char *s,
			       unsigned int n)
{
	unsigned long flags;

	spin_lock_irqsave(&con_lock, flags);
	while (n--) {
		if (*s == '\n')
			vt_putc('\r');
		vt_putc(*s++);
	}
	con_flush();
	spin_unlock_irqrestore(&con_lock, flags);
}

static struct tty_driver *fxcg_console_device(struct console *con, int *index)
{
	*index = 0;
	return fxcg_tty_driver;
}

static struct console fxcg_console = {
	.name		= "ttyFX",
	.write		= fxcg_console_write,
	.device		= fxcg_console_device,
	.flags		= CON_PRINTBUFFER | CON_ANYTIME,
	.index		= 0,
};

/* ------------------------------------------------------------ keyboard */

#define KEY(row, col)	((col) * 8 + (row))
#define NKEYS		(12 * 8)

/* Pseudo characters for the special keys */
enum {
	K_NONE = 0,
	K_SHIFT = 0x100,
	K_ALPHA,
	K_CAPS,
	K_CTRL,
};

struct keydef {
	u16 normal;		/* character or K_* */
	u16 shift;		/* yellow (SHIFT) layer, 0 = same as normal */
	char alpha;		/* red (ALPHA) letter, 0 = none */
	const char *seq;	/* escape sequence instead of a character */
	const char *shift_seq;
};

static const struct keydef keymap[NKEYS] = {
	[KEY(0, 0)] = { 0x03, 0x04 },			/* AC/ON: ^C, ^D */
	[KEY(6, 1)] = { '0', 0, 'z' },
	[KEY(5, 1)] = { '.', '=', ' ' },
	[KEY(4, 1)] = { ' ', '"', '"' },		/* x10^x */
	[KEY(3, 1)] = { '_', '%' },			/* (-) */
	[KEY(2, 1)] = { '\r' },				/* EXE */
	[KEY(6, 2)] = { '1', 0, 'u' },
	[KEY(5, 2)] = { '2', 0, 'v' },
	[KEY(4, 2)] = { '3', 0, 'w' },
	[KEY(3, 2)] = { '+', '[', 'x' },
	[KEY(2, 2)] = { '-', ']', 'y' },
	[KEY(6, 3)] = { '4', 0, 'p' },
	[KEY(5, 3)] = { '5', 0, 'q' },
	[KEY(4, 3)] = { '6', 0, 'r' },
	[KEY(3, 3)] = { '*', '{', 's' },
	[KEY(2, 3)] = { '/', '}', 't' },
	[KEY(6, 4)] = { '7', 0, 'm' },
	[KEY(5, 4)] = { '8', 0, 'n' },
	[KEY(4, 4)] = { '9', 0, 'o' },
	[KEY(3, 4)] = { 0x7f, 0, 0, NULL, "\033[3~" },	/* DEL / INS */
	[KEY(6, 5)] = { ':', 0, 'g' },			/* a b/c */
	[KEY(5, 5)] = { '@', 0, 'h' },			/* S<>D */
	[KEY(4, 5)] = { '(', 0, 'i' },
	[KEY(3, 5)] = { ')', 0, 'j' },
	[KEY(2, 5)] = { ',', 0, 'k' },
	[KEY(1, 5)] = { '\\', 0, 'l' },			/* -> */
	[KEY(6, 6)] = { '=', 0, 'a' },			/* X,theta,T */
	[KEY(5, 6)] = { '\'', 0, 'b' },			/* log */
	[KEY(4, 6)] = { '"', 0, 'c' },			/* ln */
	[KEY(3, 6)] = { '#', 0, 'd' },			/* sin */
	[KEY(2, 6)] = { '!', 0, 'e' },			/* cos */
	[KEY(1, 6)] = { '?', 0, 'f' },			/* tan */
	[KEY(6, 7)] = { K_ALPHA },
	[KEY(5, 7)] = { K_CAPS },			/* x^2 */
	[KEY(4, 7)] = { '^', '`' },
	[KEY(3, 7)] = { 0x1b },				/* EXIT: ESC */
	[KEY(2, 7)] = { 0, 0, 0, "\033[B", "\033[6~" },	/* DOWN / PgDn */
	[KEY(1, 7)] = { 0, 0, 0, "\033[C", "\033[F" },	/* RIGHT / End */
	[KEY(6, 8)] = { K_SHIFT },
	[KEY(5, 8)] = { '\t' },				/* OPTN: TAB */
	[KEY(4, 8)] = { '~' },				/* VARS */
	[KEY(3, 8)] = { K_CTRL },			/* MENU */
	[KEY(2, 8)] = { 0, 0, 0, "\033[D", "\033[H" },	/* LEFT / Home */
	[KEY(1, 8)] = { 0, 0, 0, "\033[A", "\033[5~" },	/* UP / PgUp */
	[KEY(6, 9)] = { '|', '~' },			/* F1 */
	[KEY(5, 9)] = { '>', '%' },			/* F2 */
	[KEY(4, 9)] = { '<', '`' },			/* F3 */
	[KEY(3, 9)] = { '&', '\'' },			/* F4 */
	[KEY(2, 9)] = { ';', ':' },			/* F5 */
	[KEY(1, 9)] = { '$', '"' },			/* F6 */
};

#define POLL_MS		20
#define REPEAT_DELAY	(500 / POLL_MS)
#define REPEAT_RATE	(80 / POLL_MS)

static struct {
	u16 prev[6];
	int held_key;		/* key that auto-repeats, -1 = none */
	int held_ticks;
	bool shift, alpha, alpha_lock, caps, ctrl;
	struct timer_list timer;
} kbd;

static void status_put(int *col, const char *s, bool on)
{
	u8 attr = on ? ATTR(0, 11) : STATUS_ATTR;

	while (*s && *col < COLS) {
		screen[STATUS_ROW][*col].ch = *s++;
		screen[STATUS_ROW][(*col)++].attr = attr;
	}
	if (*col < COLS) {
		screen[STATUS_ROW][*col].ch = ' ';
		screen[STATUS_ROW][(*col)++].attr = STATUS_ATTR;
	}
}

static void status_update(void)
{
	int col = 0;

	status_put(&col, "SHIFT", kbd.shift);
	status_put(&col, kbd.alpha_lock ? "A-LOCK" : "ALPHA",
		   kbd.alpha || kbd.alpha_lock);
	status_put(&col, "CAPS", kbd.caps);
	status_put(&col, "CTRL", kbd.ctrl);
	status_put(&col, "F1| F2> F3< F4& F5; F6$ OPTN=TAB", false);
	while (col < COLS) {
		screen[STATUS_ROW][col].ch = ' ';
		screen[STATUS_ROW][col++].attr = STATUS_ATTR;
	}
}

static void kbd_emit(const char *s, size_t n)
{
	tty_insert_flip_string(&fxcg_port, s, n);
	tty_flip_buffer_push(&fxcg_port);
}

/* Translate one key press into terminal input. */
static void kbd_press(int key)
{
	const struct keydef *k = &keymap[key];
	bool shift = kbd.shift, alpha = kbd.alpha || kbd.alpha_lock;
	u16 ch;
	char c;

	switch (k->normal) {
	case K_SHIFT:
		kbd.shift = !kbd.shift;
		return;
	case K_ALPHA:
		if (kbd.alpha_lock) {
			kbd.alpha_lock = false;
		} else if (kbd.shift || kbd.alpha) {
			kbd.alpha_lock = true;
			kbd.alpha = false;
		} else {
			kbd.alpha = true;
		}
		kbd.shift = false;
		return;
	case K_CAPS:
		kbd.caps = !kbd.caps;
		kbd.shift = false;
		return;
	case K_CTRL:
		kbd.ctrl = !kbd.ctrl;
		return;
	}

	/* one-shot modifiers are consumed by any other key */
	kbd.shift = false;
	kbd.alpha = false;

	if (k->seq) {
		const char *s = shift && k->shift_seq ? k->shift_seq : k->seq;

		kbd_emit(s, strlen(s));
		kbd.ctrl = false;
		return;
	}

	if ((alpha || kbd.ctrl) && k->alpha) {
		c = k->alpha;
		if (c >= 'a' && c <= 'z') {
			if (kbd.ctrl)
				c &= 0x1f;
			else if (kbd.caps ^ shift)
				c -= 'a' - 'A';
		}
		kbd.ctrl = false;
		kbd_emit(&c, 1);
		return;
	}

	if (shift && k->shift_seq) {
		kbd_emit(k->shift_seq, strlen(k->shift_seq));
		return;
	}

	ch = shift && k->shift ? k->shift : k->normal;
	if (!ch)
		return;
	c = ch;
	if (kbd.ctrl && c >= '@' && c <= '_')
		c &= 0x1f;
	kbd.ctrl = false;
	kbd_emit(&c, 1);
}

static bool key_repeats(int key)
{
	u16 n = keymap[key].normal;

	return n != K_SHIFT && n != K_ALPHA && n != K_CAPS && n != K_CTRL;
}

static void kbd_poll(struct timer_list *t)
{
	u16 now[6];
	bool changed = false;
	unsigned long flags;
	int w, b;

	for (w = 0; w < 6; w++)
		now[w] = __raw_readw(KEYSC_DATA + 2 * w);

	for (w = 0; w < 6; w++) {
		u16 pressed = now[w] & ~kbd.prev[w];

		if (!pressed)
			continue;
		for (b = 0; b < 16; b++) {
			int row = b & 7, col = 2 * w + (b >> 3);
			int key = KEY(row, col);

			if (!(pressed & (1 << b)))
				continue;
			kbd_press(key);
			changed = true;
			if (key_repeats(key)) {
				kbd.held_key = key;
				kbd.held_ticks = 0;
			}
		}
	}

	if (kbd.held_key >= 0) {
		int col = kbd.held_key / 8, row = kbd.held_key % 8;

		if (!(now[col / 2] & (1 << (row + 8 * (col & 1))))) {
			kbd.held_key = -1;
		} else if (++kbd.held_ticks >= REPEAT_DELAY &&
			   (kbd.held_ticks - REPEAT_DELAY) % REPEAT_RATE == 0) {
			kbd_press(kbd.held_key);
			changed = true;
		}
	}

	memcpy(kbd.prev, now, sizeof(now));

	if (changed) {
		spin_lock_irqsave(&con_lock, flags);
		status_update();
		con_flush();
		spin_unlock_irqrestore(&con_lock, flags);
	}

	mod_timer(&kbd.timer, jiffies + msecs_to_jiffies(POLL_MS));
}

/* ----------------------------------------------------------------- tty */

static int fxcg_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	tty->winsize.ws_row = TROWS;
	tty->winsize.ws_col = COLS;
	tty->winsize.ws_xpixel = FXCG_LCD_WIDTH;
	tty->winsize.ws_ypixel = TROWS * FONT_H;
	return tty_port_install(&fxcg_port, driver, tty);
}

static int fxcg_tty_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(&fxcg_port, tty, filp);
}

static void fxcg_tty_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(&fxcg_port, tty, filp);
}

static ssize_t fxcg_tty_write(struct tty_struct *tty, const u8 *buf,
			      size_t count)
{
	unsigned long flags;

	spin_lock_irqsave(&con_lock, flags);
	con_write_locked(buf, count);
	spin_unlock_irqrestore(&con_lock, flags);
	return count;
}

static unsigned int fxcg_tty_write_room(struct tty_struct *tty)
{
	return 4096;
}

static const struct tty_operations fxcg_tty_ops = {
	.install	= fxcg_tty_install,
	.open		= fxcg_tty_open,
	.close		= fxcg_tty_close,
	.write		= fxcg_tty_write,
	.write_room	= fxcg_tty_write_room,
};

static const struct tty_port_operations fxcg_port_ops;

static int __init fxcg_tty_init(void)
{
	struct tty_driver *drv;
	int ret;

	drv = tty_alloc_driver(1, TTY_DRIVER_REAL_RAW);
	if (IS_ERR(drv))
		return PTR_ERR(drv);

	tty_port_init(&fxcg_port);
	fxcg_port.ops = &fxcg_port_ops;

	drv->driver_name = "fxcg";
	drv->name = "ttyFX";
	drv->major = 0;
	drv->type = TTY_DRIVER_TYPE_CONSOLE;
	drv->init_termios = tty_std_termios;
	drv->init_termios.c_oflag = OPOST | ONLCR;
	tty_set_operations(drv, &fxcg_tty_ops);
	tty_port_link_device(&fxcg_port, drv, 0);

	ret = tty_register_driver(drv);
	if (ret) {
		tty_driver_kref_put(drv);
		tty_port_destroy(&fxcg_port);
		return ret;
	}
	fxcg_tty_driver = drv;

	kbd.held_key = -1;
	/* ignore keys that are still down from launching the loader */
	for (ret = 0; ret < 6; ret++)
		kbd.prev[ret] = __raw_readw(KEYSC_DATA + 2 * ret);
	timer_setup(&kbd.timer, kbd_poll, 0);
	mod_timer(&kbd.timer, jiffies + msecs_to_jiffies(POLL_MS));
	return 0;
}
device_initcall(fxcg_tty_init);

/* Called from the machine setup, long before the tty layer exists. */
void __init fxcg_console_init(void)
{
	int r, c;

	lcd_fill(palette[DEF_BG]);
	for (r = 0; r < ROWS; r++)
		for (c = 0; c < COLS; c++)
			shown[r][c] = (struct cell){ ' ', DEF_ATTR };
	vt.shown_y = -1;
	vt_reset();
	status_update();
	con_flush();
	register_console(&fxcg_console);
}
