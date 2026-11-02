/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _LINUX_RAPUYAMA_IDENTITY_H
#define _LINUX_RAPUYAMA_IDENTITY_H
#define E6_ID_OFFSET 0x800
#define E6_ID_SIZE 32

static inline int e6_id_valid(const unsigned char *s)
{
	unsigned int i, sum = 0;
	for (i = 0; i < 15; i++) {
		unsigned int d;
		if (s[i] < '0' || s[i] > '9') return 0;
		d = s[i] - '0';
		if (i & 1) { d *= 2; if (d > 9) d -= 9; }
		sum += d;
	}
	return sum && !(sum % 10);
}

static inline unsigned int e6_id_checksum(const unsigned char *p)
{
	unsigned int i, h = 2166136261U;
	for (i = 4; i < 24; i++) h = (h ^ p[i]) * 16777619U;
	return h;
}

static inline void e6_id_pack(unsigned char *p, const unsigned char *s)
{
	unsigned int i, h;
	for (i = 0; i < E6_ID_SIZE; i++) p[i] = 0;
	if (!s || !e6_id_valid(s)) return;
	p[0] = 'E'; p[1] = '6'; p[2] = 'I'; p[3] = 'D';
	p[4] = 1; p[5] = 15;
	for (i = 0; i < 15; i++) p[8+i] = s[i];
	h = e6_id_checksum(p);
	for (i = 0; i < 4; i++) p[24+i] = h >> (8*i);
}
static inline int e6_id_unpack(const unsigned char *p, unsigned char *s)
{
	unsigned int i, h = 0;
	if (p[0]!='E' || p[1]!='6' || p[2]!='I' || p[3]!='D' || p[4]!=1 || p[5]!=15 || p[6] || p[7] || p[23]) return 0;
	for (i = 28; i < 32; i++) if (p[i]) return 0;
	for (i = 0; i < 4; i++) h |= (unsigned int)p[24+i] << (8*i);
	if (h != e6_id_checksum(p) || !e6_id_valid(p+8)) return 0;
	for (i = 0; i < 15; i++) s[i] = p[8+i];
	return 1;
}

static inline int e6_id_info_error(const unsigned char *p, unsigned int size,
		unsigned char resource, unsigned char tid, unsigned char *s)
{
	unsigned int n, off = 12, found = 0, where = 0;
	if (size < 12) return 1;
	if (p[3]!=resource || p[8]!=tid) return 2;
	if (p[9]!=1 || p[10]) return 3;
	if ((unsigned int)(p[4] | (unsigned int)p[5]<<8)+6 != size &&
	    (unsigned int)((unsigned int)p[4]<<8 | p[5])+6 != size) return 4;
	for (n = 0; n < p[11]; n++) {
		unsigned int len;
		if (off+2 > size) return 5;
		len = p[off+1];
		if (len < 4 || len > size-off) return 5;
		if (p[off]==0x41) {
			if (found) return 6;
			if (len!=20 || p[off+3]!=16 || p[off+19]) return 7;
			if (!e6_id_valid(p+off+4)) return 8;
			found = 1; where = off+4;
		}
		off += len;
	}
	if (!found || off!=size) return 9;
	for (n = 0; n < 15; n++) s[n] = p[where+n];
	return 0;
}
static inline int e6_id_info(const unsigned char *p, unsigned int size,
		unsigned char resource, unsigned char tid, unsigned char *s)
{
	return e6_id_info_error(p,size,resource,tid,s)==0;
}
#endif
