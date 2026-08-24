#ifndef SETTINGS_H
#define SETTINGS_H

#include "INCLUDER.h"
#include <cstdarg>

typedef struct
{
		uint32_t can_id;
		uint32_t offset;
		uint32_t length;
} CANJsonIndexEntry;

class CANJsonIndex
{
	public:
		CANJsonIndexEntry *entries = nullptr;
		size_t num_entries = 0;
		size_t capacity = 0;

		CANJsonIndex()
		{
			reserve(10);
		}

		~CANJsonIndex()
		{
			delete[] entries;
		}

		void reserve(size_t new_cap)
				{
			if (new_cap <= capacity)
				return;
			CANJsonIndexEntry *new_entries = new CANJsonIndexEntry[new_cap];
			for (size_t i = 0; i < num_entries; ++i)
				new_entries[i] = entries[i];
			delete[] entries;
			entries = new_entries;
			capacity = new_cap;
		}

		void clear()
		{
			num_entries = 0;
		}

		// Load all index entries from .bin file (each entry: 12 bytes, little-endian)
		bool loadFromBin(const std::string &path)
				{
			FIL file;
			if (f_open(&file, (const TCHAR*) path.c_str(), FA_READ) != FR_OK)
				return false;
			clear();
			uint8_t buf[12];
			UINT br;
			while (f_read(&file, buf, 12, &br) == FR_OK && br == 12)
			{
				if (num_entries >= capacity)
					reserve(capacity + 10);
				CANJsonIndexEntry &e = entries[num_entries++];
				e.can_id = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
				e.offset = buf[4] | (buf[5] << 8) | (buf[6] << 16) | (buf[7] << 24);
				e.length = buf[8] | (buf[9] << 8) | (buf[10] << 16) | (buf[11] << 24);
			}
			f_close(&file);
			return num_entries > 0;
		}

		// Find entry by CAN ID
		const CANJsonIndexEntry* find(uint32_t can_id) const
				{
			for (size_t i = 0; i < num_entries; ++i)
					{
				if (entries[i].can_id == can_id)
					return &entries[i];
			}
			return nullptr;
		}

		// Utility: Read JSON string for a given offset/length (from bin index) from the JSON file
		// Returns true and fills jsonStr if found
		static bool readJsonByOffset(const std::string &jsonPath, uint32_t offset, uint32_t length, std::string &jsonStr)
				{
			FIL file;
			if (f_open(&file, (const TCHAR*) jsonPath.c_str(), FA_READ) != FR_OK)
				return false;
			if (f_lseek(&file, offset) != FR_OK)
					{
				f_close(&file);
				return false;
			}
			jsonStr.clear();
			char *buf = new char[length + 1];
			UINT br;
			if (f_read(&file, buf, length, &br) == FR_OK && br == length)
					{
				buf[length] = 0;
				jsonStr.assign(buf, length);
				delete[] buf;
				f_close(&file);
				return true;
			}
			delete[] buf;
			f_close(&file);
			return false;
		}
};

#define MAX_CAN_MESSAGES 16
#define MAX_SIGNALS_PER_MESSAGE 16

// On-disk binary layout exported by the DBC web tool.
// Strings are fixed-length, zero-padded ASCII; CRC32 covers everything except the crc32 field itself.
constexpr size_t CAN_STRUCT_NAME_MAX = 24;
constexpr size_t CAN_STRUCT_SIGNAL_NAME_MAX = 24;

#pragma pack(push, 1)
typedef struct
{
		char name[CAN_STRUCT_SIGNAL_NAME_MAX];
		uint16_t start;
		uint8_t length;
		uint8_t byteOrder;
		uint8_t is_signed;
		uint8_t reserved[3];
		float scale;
		float offset;
		float minimum;
		float maximum;
} __attribute__((packed)) PackedCANSignal;

typedef struct
{
		uint32_t can_id;
		uint8_t name_len;
		uint8_t num_signals;
		uint8_t data_len;
		uint8_t reserved0;
		char name[CAN_STRUCT_NAME_MAX];
		PackedCANSignal signals[MAX_SIGNALS_PER_MESSAGE];
		uint8_t data[16];
		uint32_t crc32;
} __attribute__((packed)) PackedCANMessage;
#pragma pack(pop)

// Expected on-disk size from exporter. Struct must be <= this.
constexpr size_t PACKED_CAN_MESSAGE_SIZE = 820;
static_assert(sizeof(PackedCANMessage) <= PACKED_CAN_MESSAGE_SIZE, "PackedCANMessage larger than exported size");

typedef struct
{
		char name[24];
		uint16_t start;
		uint8_t length;
		uint8_t byteOrder; // 0: little_endian, 1: big_endian
		uint8_t is_signed;
		float scale;
		float offset;
		float minimum;
		float maximum;
} CANSignal;

typedef struct
{
		uint32_t can_id;
		char name[24];
		CANSignal signals[MAX_SIGNALS_PER_MESSAGE];
		uint8_t num_signals;
		uint8_t data[16];
		uint8_t data_len;
} CANMessage;

typedef struct
{
		CANMessage messages[MAX_CAN_MESSAGES];
		uint8_t num_messages;
} CANSettings;

// Forward declaration for packed-struct reader; crcError optional flag set true on CRC mismatch
bool readAndParseCANMessage(const char *dataPath, uint32_t offset, uint32_t length, CANMessage &outMsg);

inline uint32_t crc32_le(const uint8_t *data, size_t len)
		{
	uint32_t crc = 0xFFFFFFFFu;
	for (size_t i = 0; i < len; ++i)
			{
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; ++bit)
				{
			uint32_t mask = -(crc & 1u);
			crc = (crc >> 1) ^ (0xEDB88320u & mask);
		}
	}
	return ~crc;
}

inline uint8_t boundedStrnlen(const char *s, size_t maxLen)
		{
	uint8_t len = 0;
	while (len < maxLen && s[len] != 0)
		++len;
	return len;
}

// Parses CAN settings from the binary struct file using the exported index (.idx)
inline bool parseCANSettingsFromIndex(const std::string &indexPath, const std::string &dataPath, CANSettings &out, bool *crcErrorOut = nullptr)
		{
	CANJsonIndex idx;
	if (!idx.loadFromBin(indexPath))
		return false;

	bool crcFail = false;
	out.num_messages = 0;
	for (size_t i = 0; i < idx.num_entries && out.num_messages < MAX_CAN_MESSAGES; ++i)
			{
		const CANJsonIndexEntry &entry = idx.entries[i];
		CANMessage &msg = out.messages[out.num_messages];
		bool crcErrLocal = false;
		if (readAndParseCANMessage(dataPath.c_str(), entry.offset, entry.length, msg))
			++out.num_messages;
		if (crcErrLocal)
			crcFail = true;
	}
	if (crcErrorOut)
		*crcErrorOut = crcFail;
	return out.num_messages > 0;
}

// Print a parsed CANMessage using SWO_PRINTER (array version)
void printCANMessage(const CANMessage &msg, SWO_PRINTER &printer)
		{
	printer.snprint("0x%08lX : %d : %s \r\n", (unsigned long) msg.can_id, (int) msg.num_signals, msg.name);
//    printer.Print("Name: " + String(msg.name) + "\r\n");
//	printer.snprint("Signals: %d\r\n");
	for (uint8_t i = 0; i < msg.num_signals; ++i)
			{
		const CANSignal &s = msg.signals[i];
		printer.snprint("  [%d] %s: %u,%u,%u,%u,%u,%u,%u,%u\r\n", (int) i, s.name, s.start, s.length, s.byteOrder, s.is_signed ? 1 : 0, (unsigned long) s.scale, (unsigned long) s.offset, (unsigned long) s.minimum, (unsigned long) s.maximum);
	}
	if (msg.data_len > 0)
			{
		printer.Print("Data: ");
		for (uint8_t i = 0; i < msg.data_len; ++i)
				{
			printer.snprint("%02X ", msg.data[i]);
		}
		printer.Print("\r\n");
	}
}

// Extract bitfield up to 64 bits. Supports little-endian (Intel) and big-endian (Motorola) layouts.
static uint64_t extractBits(const uint8_t *data, uint8_t start, uint8_t len, uint8_t byteOrder)
		{
	if (len == 0)
		return 0;
	uint64_t val = 0;
	if (byteOrder == 0)
			{
		// Little-endian / Intel: start = LSB-based bit index
		for (uint8_t i = 0; i < len; ++i)
				{
			uint32_t bitIndex = (uint32_t) start + i;
			uint32_t byteIndex = bitIndex / 8;
			uint32_t bitInByte = bitIndex % 8;
			if (byteIndex >= 16)
				break;
			if ((data[byteIndex] >> bitInByte) & 1u)
				val |= (1ULL << i);
		}
	}
	else
	{
		// Big-endian / Motorola: start is index of the most significant bit
		for (uint8_t i = 0; i < len; ++i)
				{
			// source bit index decreases
			int32_t source_bit = (int32_t) start - (int32_t) i;
			if (source_bit < 0)
				break;
			uint32_t byteIndex = (uint32_t) source_bit / 8u;
			uint32_t bitInByte = (uint32_t) source_bit % 8u;
			if (byteIndex >= 16)
				break;
			uint32_t actual_bit = 7u - bitInByte; // MSB-first within byte
			if ((data[byteIndex] >> actual_bit) & 1u)
				val |= (1ULL << i);
		}
	}
	return val;
}

// Helper to safely append formatted text into buffer, updating offset.
static void appendToBuf(char *outBuf, size_t outSize, int *offPtr, const char *fmt, ...)
		{
	int off = *offPtr;
	if (off >= (int) outSize - 1)
		return;
	size_t rem = outSize - (size_t) off;
	va_list ap;
	va_start(ap, fmt);
	int need = vsnprintf(outBuf + off, rem, fmt, ap);
	va_end(ap);
	if (need < 0)
		return;
	if ((size_t) need >= rem)
			{
		*offPtr = (int) outSize - 1;
		outBuf[*offPtr] = '\0';
	}
	else
	{
		*offPtr = off + need;
	}
}

// Build a JSON string containing decoded signal meanings for a CANMessage.
// Caller provides a buffer and size; function writes a NUL-terminated string.

// Builds a single JSON object for one CANMessage (no outer array).
// This is used by the batcher and also kept for backward compatibility.
static void buildCANMeaningJsonObject(const CANMessage &msg, char *outBuf, size_t outSize)
		{
	int off = 0;
	appendToBuf(outBuf, outSize, &off, "{\"can_id\":\"0x%08lX\",\"name\":\"%s\",\"signals\":[", (unsigned long) msg.can_id, msg.name);
	for (uint8_t i = 0; i < msg.num_signals; ++i)
			{
		if (off >= (int) outSize - 64)
			break;
		const CANSignal &s = msg.signals[i];
		uint64_t raw = extractBits(msg.data, s.start, s.length, s.byteOrder);
		int64_t signedRaw = (int64_t) raw;
		if (s.is_signed && s.length > 0 && s.length < 64)
				{
			uint64_t signbit = 1ULL << (s.length - 1);
			if (raw & signbit)
					{
				uint64_t mask = (~0ULL) << s.length;
				signedRaw = (int64_t) (raw | mask);
			}
		}
		double phys = (double) signedRaw * (double) s.scale + (double) s.offset;
		if (i > 0)
			appendToBuf(outBuf, outSize, &off, ",");
		appendToBuf(outBuf, outSize, &off, "{\"%s\":%.6g}", s.name, phys);
	}
	appendToBuf(outBuf, outSize, &off, "]}");
}

// Backward-compatible entry point: builds a single-message JSON object.
void buildCANMeaningJson(const CANMessage &msg, char *outBuf, size_t outSize)
		{
	buildCANMeaningJsonObject(msg, outBuf, outSize);
}

// Double-buffered batch builder for CAN meaning JSON.
// Produces a JSON array of per-message objects, capped to payloadMax bytes.
// When appending a message would exceed payloadMax, it finalizes the current buffer (']')
// and switches to the other buffer for continued filling.
class CANMeaningJsonBatcher
{
	public:
		CANMeaningJsonBatcher(char *buf0, char *buf1, size_t payloadMax)
			: payloadMax(payloadMax)
		{
			bufs[0] = buf0;
			bufs[1] = buf1;
			resetBuffer(0);
			resetBuffer(1);
			active = 0;
		}

		// Appends msg to active array.
		// If a flush is needed, sets flushBuf/flushLen to the finalized JSON array from the previous buffer.
		// Returns true if the message was appended (to either the current buffer or after switching).
		bool append(const CANMessage &msg, const char **flushBuf, size_t *flushLen)
		{
			if (flushBuf)
				*flushBuf = nullptr;
			if (flushLen)
				*flushLen = 0;

			if (tryAppendTo(active, msg))
				return true;

			// finalize current active and switch (only if it already has content)
			if (count[active] > 0)
			{
				finalize(active);
				if (flushBuf)
					*flushBuf = bufs[active];
				if (flushLen)
					*flushLen = strlen(bufs[active]);
			}

			active = 1 - active;
			resetBuffer(active);

			// retry append on the new active buffer
			if (tryAppendTo(active, msg))
				return true;

			// Message can't fit even in an empty buffer -> emit minimal object.
			resetBuffer(active);
			appendMinimalTooLarge(active, msg);
			return false;
		}

		// Finalize and return current active buffer if it has at least one element.
		bool flushCurrent(const char **flushBuf, size_t *flushLen)
		{
			if (count[active] == 0)
				return false;
			finalize(active);
			if (flushBuf)
				*flushBuf = bufs[active];
			if (flushLen)
				*flushLen = strlen(bufs[active]);
			resetBuffer(active);
			return true;
		}

	private:
		char *bufs[2] { nullptr, nullptr };
		size_t payloadMax = 0;
		int active = 0;
		int off[2] { 0, 0 };
		uint16_t count[2] { 0, 0 };

		void resetBuffer(int idx)
		{
			off[idx] = 0;
			count[idx] = 0;
			if (bufs[idx] == nullptr || payloadMax == 0)
				return;
			bufs[idx][0] = '[';
			bufs[idx][1] = '\0';
			off[idx] = 1;
		}

		void finalize(int idx)
		{
			if (bufs[idx] == nullptr || payloadMax == 0)
				return;
			// ensure there's room for closing bracket
			if (off[idx] >= (int) payloadMax - 2)
			{
				// hard clamp
				off[idx] = (int) payloadMax - 2;
				bufs[idx][off[idx]] = '\0';
			}
			bufs[idx][off[idx]++] = ']';
			bufs[idx][off[idx]] = '\0';
		}

		bool tryAppendTo(int idx, const CANMessage &msg)
		{
			if (bufs[idx] == nullptr || payloadMax == 0)
				return false;

			int oldOff = off[idx];

			// reserve space for closing ']' and NUL
			if (oldOff >= (int) payloadMax - 2)
				return false;

			// Build object in scratch first, so we never partially modify the target buffer.
			char tmp[1024];
			buildCANMeaningJsonObject(msg, tmp, sizeof(tmp));
			size_t objLen = strlen(tmp);
			if (objLen >= sizeof(tmp) - 1)
				return false;

			size_t extra = objLen + ((count[idx] > 0) ? 1u : 0u); // + comma if needed
			if ((size_t) oldOff + extra + 2 > payloadMax)
				return false;

			// Now it is guaranteed to fit: write comma (optional) + object.
			if (count[idx] > 0)
			{
				bufs[idx][off[idx]++] = ',';
				bufs[idx][off[idx]] = '\0';
			}

			memcpy(bufs[idx] + off[idx], tmp, objLen);
			off[idx] += (int) objLen;
			bufs[idx][off[idx]] = '\0';
			count[idx]++;
			return true;
		}

		void appendMinimalTooLarge(int idx, const CANMessage &msg)
		{
			if (bufs[idx] == nullptr || payloadMax == 0)
				return;
			// Always fits in 2048; keep the array open and let finalize() add ']'.
			int localOff = off[idx];
			appendToBuf(bufs[idx], payloadMax, &localOff,
					"{\"can_id\":%x,\"name\":\"%s\",\"too_large\":1}",
					(unsigned long) msg.can_id, msg.name);
			off[idx] = localOff;
			count[idx] = 1;
		}
};

// Overload: append into a JSON-array batcher (<= payloadMax per send).
inline bool buildCANMeaningJson(const CANMessage &msg, CANMeaningJsonBatcher &batcher, const char **flushBuf, size_t *flushLen)
		{
	return batcher.append(msg, flushBuf, flushLen);
}

// Global CAN binary (struct) file handle for optimized access
FIL g_jsonFile;
bool g_jsonFileOpen;

bool openGlobalJsonFile(const std::string &dataPath)
		{
	if (g_jsonFileOpen)
		return true;
	if (f_open(&g_jsonFile, (const TCHAR*) dataPath.c_str(), FA_READ) == FR_OK)
			{
		g_jsonFileOpen = true;
		return true;
	}
	return false;
}

void closeGlobalJsonFile()
{
	if (g_jsonFileOpen)
	{
		f_close(&g_jsonFile);
		g_jsonFileOpen = false;
	}
}

bool readAndParseCANMessage(const char *dataPath, uint32_t offset, uint32_t length, CANMessage &outMsg)
		{
	// Use global file if open, else open/close locally
	FIL *filePtr = nullptr;
	bool localOpen = false;
	if (g_jsonFileOpen)
	{
		filePtr = &g_jsonFile;
	}
	else
	{
		filePtr = new FIL;
		if (f_open(filePtr, (const TCHAR*) dataPath, FA_READ) != FR_OK)
				{
			delete filePtr;
			return false;
		}
		localOpen = true;
	}
	if (f_lseek(filePtr, offset) != FR_OK)
			{
		if (localOpen)
		{
			f_close(filePtr);
			delete filePtr;
		}
		return false;
	}
	if (length < PACKED_CAN_MESSAGE_SIZE)
			{
		if (localOpen)
		{
			f_close(filePtr);
			delete filePtr;
		}
		return false;
	}

	uint8_t rawBuf[PACKED_CAN_MESSAGE_SIZE];
	UINT br = 0;
	bool ok = false;

	if (f_read(filePtr, rawBuf, PACKED_CAN_MESSAGE_SIZE, &br) == FR_OK && br == PACKED_CAN_MESSAGE_SIZE)
			{
		PackedCANMessage packed { };
		memcpy(&packed, rawBuf, sizeof(packed));
		uint32_t crc = crc32_le(rawBuf, PACKED_CAN_MESSAGE_SIZE - sizeof(packed.crc32));
		if (crc == packed.crc32)
				{
//			memset(&outMsg, 0, sizeof(CANMessage));
			outMsg.can_id = packed.can_id;
			uint8_t name_len = packed.name_len;
			if (name_len >= sizeof(outMsg.name))
				name_len = sizeof(outMsg.name) - 1;
			memcpy(outMsg.name, packed.name, name_len);
			outMsg.name[name_len] = 0;

			uint8_t numSignals = packed.num_signals;
			if (numSignals > MAX_SIGNALS_PER_MESSAGE)
				numSignals = MAX_SIGNALS_PER_MESSAGE;
			outMsg.num_signals = numSignals;

			for (uint8_t i = 0; i < numSignals; ++i)
					{
				const PackedCANSignal &src = packed.signals[i];
				CANSignal &dst = outMsg.signals[i];
				uint8_t sigNameLen = boundedStrnlen(src.name, sizeof(src.name));
				if (sigNameLen >= sizeof(dst.name))
					sigNameLen = sizeof(dst.name) - 1;
				memcpy(dst.name, src.name, sigNameLen);
				dst.name[sigNameLen] = 0;
				dst.start = src.start;
				dst.length = src.length;
				dst.byteOrder = src.byteOrder;
				dst.is_signed = src.is_signed;
				dst.scale = src.scale;
				dst.offset = src.offset;
				dst.minimum = src.minimum;
				dst.maximum = src.maximum;
			}

			// Read data length from the parsed packed struct (more robust)

//			outMsg.data_len = num_signals;
//			if (num_signals > 0)
//				memcpy(outMsg.data, packed.data, dataLen);
			ok = true;
		}

	}

	if (localOpen)
	{
		f_close(filePtr);
		delete filePtr;
	}
	return ok;
}
CANJsonIndex g_canIndex;

// Test function: Reads CAN index and JSON, prints CAN message info for each entry
void test_CANJsonIndex_read(const std::string &binPath, const std::string &dataPath, SWO_PRINTER &printer)
		{

	if (!g_canIndex.loadFromBin(binPath))
			{
		printer.Print("Failed to load CAN index file\r\n");
		return;
	}
	printer.snprint("Loaded %u CAN index entries\r\n", (unsigned) g_canIndex.num_entries);

	return;
	for (size_t i = 0; i < g_canIndex.num_entries; ++i)
			{
		const CANJsonIndexEntry &entry = g_canIndex.entries[i];
		CANMessage msg;
		if (readAndParseCANMessage(dataPath.c_str(), entry.offset, entry.length, msg))
				{
			printCANMessage(msg, printer);
		}
		else
		{
			printer.snprint("Failed to read/parse CAN message at offset %lu len %lu (CAN ID 0x%08lX)\r\n", (unsigned long) entry.offset, (unsigned long) entry.length, (unsigned long) entry.can_id);
		}
	}
}

#endif // SETTINGS_H
