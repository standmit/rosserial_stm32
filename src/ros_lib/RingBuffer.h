#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include "stdint.h"

typedef uint16_t RB_SizeType;
typedef uint8_t RB_DataType;

template<RB_SizeType SIZE>
class RingBuffer {
	protected:
		RB_DataType data[SIZE];
		RB_SizeType read_i = 0;
		RB_SizeType write_i = 0;
		RB_SizeType lastpart_end = read_i;
		RB_SizeType freespace_end = write_i;

	public:
		inline RB_SizeType CountRW()const {
			return (write_i - read_i);
		}

		inline RB_SizeType Count()const {
			if (read_i <= write_i) {
				return CountRW();
			} else {
				return (SIZE - read_i + write_i);
			}
		}

		inline RB_SizeType FreeSpace()const {
			return (SIZE - Count() - 1);
		}

		bool Write(RB_DataType* buffer, RB_SizeType count) {
			if (FreeSpace() >= count) {
				if (write_i >= read_i) {
					RB_SizeType part_size = SIZE - write_i;
					if (count <= part_size) {
						memcpy(&data[write_i], buffer, count);
						write_i = (write_i + count) % SIZE;
					} else {
						memcpy(&data[write_i], buffer, part_size);
						memcpy(&data[0], buffer + part_size, count - part_size);
						write_i = count - part_size;
					}
				} else {
					memcpy(&data[write_i], buffer, count);
					write_i += count;
				}

				return true;
			} else {
				return false;
			}
		}

		bool Read(RB_DataType& buffer) {
			if (read_i != write_i) {
				buffer = data[read_i];
				read_i++;
				if (read_i >= SIZE)
					read_i = 0;
				return true;
			} else {
				return false;
			}
		}

		RB_SizeType GetLastPart(RB_DataType* &buf) {
			buf = &data[read_i];
			RB_SizeType sz;
			if (read_i > write_i) {
				sz = SIZE - read_i;
				lastpart_end = 0;
			} else {
				sz = CountRW();
				lastpart_end = write_i;
			}
			return sz;
		}

		inline void DropLastPart() {
			read_i = lastpart_end;
		}

		RB_SizeType GetFreeSpace(RB_DataType* &buf) {
			buf = &data[write_i];
			RB_SizeType sz;
			if (write_i < read_i) {
				sz = read_i - 1 - write_i;
				freespace_end = read_i - 1;
			} else {
				sz = SIZE - write_i;
				freespace_end = 0;
			}
			return sz;
		}

		inline void OccupyFreeSpace(const int32_t count = -1) {
			if (count == -1) {
				write_i = freespace_end;
			} else {
				write_i = (write_i + count) % SIZE;
			}
		}
};


#endif /* RINGBUFFER_H_ */
