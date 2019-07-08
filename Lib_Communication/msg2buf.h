#pragma once

#include "DataMsg.h"

/*! \brief Flatbuffers-based serialization tool for DataMsg format
*
*/
class Buffer {
public:
	Buffer(); /*!< Constructor for zero length buffer */
	Buffer(const unsigned char * buf_, size_t size_); /*!< Constructor from given pointer */
	Buffer(const Buffer& buf0); /*!< Copy constructor */
	Buffer(Buffer&& o); /*!< Copy constructor */

	Buffer(const DataMsg& data); /*!< Initialize from a DataMsg instance */

	DataMsg ExtractDataMsg() const; /*!< Extract DataMsg from the serialized data*/
	
	~Buffer(); /*!< Destructor */

	Buffer& operator=(const Buffer& buf0); /*!< Assign operator */

	bool isNull() const; /*!< Returns if the buffer is zero length buffer */

	void print() const; /*!< Print relevant informations */

	size_t Size() const; /*!< Returns the size of the buffer */

	const unsigned char* Buf() const; /*!< Returns the pointer of the data*/

private:
	void _allocandcopy(const unsigned char * buf_);

	unsigned char* buf;

	size_t size;
};
