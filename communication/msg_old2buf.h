#pragma once

#include "DataMsg.h"

namespace SF {

	/*! \brief Flatbuffers-based serialization tool for DataMsg format
	*
	*/
	class Buffer_old {
	public:
		Buffer_old(); /*!< Constructor for zero length buffer */
		Buffer_old(const unsigned char * buf_, size_t size_); /*!< Constructor from given pointer */
		Buffer_old(const Buffer_old& buf0); /*!< Copy constructor */
		Buffer_old(Buffer_old&& o); /*!< Copy constructor */

		Buffer_old(const DataMsg& data); /*!< Initialize from a DataMsg instance */

		~Buffer_old(); /*!< Destructor */

		Buffer_old& operator=(const Buffer_old& buf0); /*!< Assign operator */

		bool isNull() const; /*!< Returns if the buffer is zero length buffer */

		void print() const; /*!< Print relevant informations */

		size_t Size() const; /*!< Returns the size of the buffer */

		const unsigned char* Buf() const; /*!< Returns the pointer of the data*/

	private:
		void _allocandcopy(const unsigned char * buf_);

		unsigned char* buf;

		size_t size;
	};

	bool ExtractBufIf(void* buf, size_t size, DataMsg& data);

}
