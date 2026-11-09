/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef RAPUYAMA_SECURE_HEAP_H
#define RAPUYAMA_SECURE_HEAP_H

static inline unsigned secure_heap_word(const unsigned char *p)
{
	return p[0] | (unsigned)p[1] << 8 | (unsigned)p[2] << 16 | (unsigned)p[3] << 24;
}

static inline int secure_heap_room(const unsigned char *heap, unsigned head, unsigned request, unsigned response)
{
	unsigned offset = 0x2020, selected = 0, reply_fits = 0;

	if (head && head != 0xc6702020)
		return 0;
	request = (request + 3) & ~3U;
	response = (response + 3) & ~3U;
	for (;;) {
		unsigned next = head ? secure_heap_word(heap + offset) : 0;
		unsigned size = head ? secure_heap_word(heap + offset + 4) : 0x5fd4;
		unsigned used = head ? heap[offset + 8] : 0;
		unsigned end;

		if (used > 1 || size > 0x8000 - offset - 12)
			return 0;
		end = offset + 12 + size;
		if (next && ((next & 3) || next != 0xc6700000 + end || end > 0x8000 - 12))
			return 0;
		if (!used) {
			if (!selected && size >= request) {
				if (offset + 12 + request >= 0x8000)
					return 0;
				selected = 1;
				if (size - request > 15 && size - request - 12 >= response &&
				    (size - request - 12 - response > 0 || end < 0x8000))
					reply_fits = 1;
			} else if (size >= response && (size - response > 0 || end < 0x8000)) {
				reply_fits = 1;
			}
		}
		if (!next)
			return selected && reply_fits;
		offset = next - 0xc6700000;
	}
}
#endif