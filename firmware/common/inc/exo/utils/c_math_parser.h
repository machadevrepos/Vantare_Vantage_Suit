// C_MATH_PARSER.h
// STM32-friendly, header-only math expression parser (recursive-descent)
// - No C++ exceptions in core parser
// - Core parser uses C strings (const char*) and C library functions
// - Provides a small C API and an optional C++ wrapper that mimics the original API
// Supports: + - * / ^ (power), parentheses, unary +/-, and decimal numbers
// Exponent operator (^) is right-associative and exponent magnitude is limited to |exp| <= 12

#ifndef C_MATH_PARSER_H
#define C_MATH_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/*
 * C result structure (safe for embedded)
 * - error message is a small fixed buffer to avoid dynamic allocation
 */
typedef struct {
	double value;
	bool ok;            /* true on success */
	char error[64];     /* empty on success */
} CMathResult_t;

/* C API: parse expression from a NUL-terminated C string */
static inline CMathResult_t CMathParser_parse_c(const char *expr);

#ifdef __cplusplus
extern "C" {
/* expose the C API symbol for linking from C files if needed */
CMathResult_t CMathParser_parse_c(const char *expr);
}
#endif

/* Implementation (header-only) */

/* Internal parser state (C style) */
typedef struct {
	const char *s;
	size_t pos;
	size_t len;
	char *errbuf;
	size_t errlen;
	bool error;
} ParserState;

static inline void set_error(ParserState *p, const char *msg) {
	p->error = true;
	if (p->errbuf && p->errlen) {
		strncpy(p->errbuf, msg, p->errlen - 1);
		p->errbuf[p->errlen - 1] = '\0';
	}
}

static inline void skipws(ParserState *p) {
	while (p->pos < p->len && isspace((unsigned char)p->s[p->pos])) p->pos++;
}

static double parseExpression(ParserState *p);
static double parseTerm(ParserState *p);
static double parseFactor(ParserState *p);
static double parsePrimary(ParserState *p);

static double parseExpression(ParserState *p) {
	double v = parseTerm(p);
	if (p->error) return 0.0;
	while (true) {
		skipws(p);
		if (p->pos >= p->len) break;
		char op = p->s[p->pos];
		if (op == '+' || op == '-') {
			p->pos++;
			double rhs = parseTerm(p);
			if (p->error) return 0.0;
			v = (op == '+') ? (v + rhs) : (v - rhs);
		} else break;
	}
	return v;
}

static double parseTerm(ParserState *p) {
	double v = parseFactor(p);
	if (p->error) return 0.0;
	while (true) {
		skipws(p);
		if (p->pos >= p->len) break;
		char op = p->s[p->pos];
		if (op == '*' || op == '/') {
			p->pos++;
			double rhs = parseFactor(p);
			if (p->error) return 0.0;
			if (op == '/') {
				if (rhs == 0.0) { set_error(p, "division by zero"); return 0.0; }
				v /= rhs;
			} else {
				v *= rhs;
			}
		} else break;
	}
	return v;
}

static double parseFactor(ParserState *p) {
	double base = parsePrimary(p);
	if (p->error) return 0.0;
	skipws(p);
	if (p->pos < p->len && p->s[p->pos] == '^') {
		p->pos++;
		double exponent = parseFactor(p);
		if (p->error) return 0.0;
		if (!isfinite(base) || !isfinite(exponent)) { set_error(p, "non-finite pow argument"); return 0.0; }
		if (fabs(exponent) > 12.0) { set_error(p, "exponent magnitude > 12 not allowed"); return 0.0; }
		double res = pow(base, exponent);
		if (!isfinite(res)) { set_error(p, "result not finite"); return 0.0; }
		return res;
	}
	return base;
}

static double parsePrimary(ParserState *p) {
	skipws(p);
	if (p->pos >= p->len) { set_error(p, "unexpected end of expression"); return 0.0; }
	char c = p->s[p->pos];
	if (c == '(') {
		p->pos++;
		double v = parseExpression(p);
		if (p->error) return 0.0;
		skipws(p);
		if (p->pos >= p->len || p->s[p->pos] != ')') { set_error(p, "missing closing parenthesis"); return 0.0; }
		p->pos++;
		return v;
	}
	if (c == '+' || c == '-') {
		p->pos++;
		double v = parsePrimary(p);
		if (p->error) return 0.0;
		return (c == '-') ? -v : v;
	}

	/* use strtod for robust number parsing */
	const char *start = p->s + p->pos;
	char *endptr = NULL;
	errno = 0;
	double val = strtod(start, &endptr);
	if (endptr == start) { set_error(p, "expected number or parenthesis"); return 0.0; }
	if (errno != 0) { set_error(p, "invalid number format"); return 0.0; }
	p->pos = (size_t)(endptr - p->s);
	return val;
}

static inline CMathResult_t CMathParser_parse_c(const char *expr) {
	CMathResult_t result;
	result.value = 0.0;
	result.ok = false;
	result.error[0] = '\0';
	if (!expr) { strncpy(result.error, "null input", sizeof(result.error) - 1); return result; }
	ParserState st;
	st.s = expr;
	st.pos = 0;
	st.len = strlen(expr);
	st.errbuf = result.error;
	st.errlen = sizeof(result.error);
	st.error = false;

	skipws(&st);
	if (st.pos >= st.len) { set_error(&st, "empty expression"); return result; }

	double v = parseExpression(&st);
	if (st.error) return result;

	skipws(&st);
	if (st.pos != st.len) { set_error(&st, "unexpected characters at end"); return result; }

	result.value = v;
	result.ok = true;
	result.error[0] = '\0';
	return result;
}

#ifdef __cplusplus
/* C++ wrapper to preserve original API shape (optional) */
#include <string>
namespace CUSTOM {
	struct CMathResult {
		double value = 0.0;
		bool ok = false;
		std::string error; // empty on success
	};

	inline CMathResult parse(const std::string &expr) {
		CMathResult_t r = CMathParser_parse_c(expr.c_str());
		CMathResult out;
		out.value = r.value;
		out.ok = r.ok;
		out.error = r.error;
		return out;
	}
}
#endif /* __cplusplus */

#endif /* C_MATH_PARSER_H */

