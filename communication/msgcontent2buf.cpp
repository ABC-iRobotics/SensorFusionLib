#include "msgcontent2buf.h"
#include <flatbuffers/flatbuffers.h>
#include "msgcontent_generated.h"

using namespace SF;

bool SF::VerifyDataMsgContent(void * buf, int length) {
	flatbuffers::Verifier v((uint8_t*)buf, length);
	auto msg = DataMsgContentNameSpace::GetMsg(buf);
	return msg->Verify(v);
}

DataMsg SF::InitDataMsg(void* buf, OperationType source, unsigned char ID, DataType type) {
	auto msg = DataMsgContentNameSpace::GetMsg(buf);
	auto timestamp_in_us = msg->timestamp_in_us();
	DataMsg data = DataMsg(ID, type, source, Time(std::chrono::microseconds(timestamp_in_us)));
	if (flatbuffers::IsFieldPresent(msg, DataMsgContentNameSpace::Msg::VT_VALUE_VECTOR)) {
		auto v = msg->value_vector();
		size_t N = v->size();
		auto value = Eigen::VectorXd(N);
		for (unsigned int n = 0; n < N; n++)
			value[n] = v->Get(n);
		data.SetValueVector(value);
	}
	if (flatbuffers::IsFieldPresent(msg, DataMsgContentNameSpace::Msg::VT_VARIANCE_MATRIX)) {
		auto v = msg->variance_matrix();
		size_t K = v->size();
		size_t N = static_cast<size_t>((sqrt(8 * K + 1) - 1) / 2);
		auto variance = Eigen::MatrixXd(N, N);
		int k = 0;
		for (unsigned int i = 0; i < N; i++)
			for (unsigned int j = i; j < N; j++) {
				variance(i, j) = v->Get(k);
				variance(j, i) = variance(i, j);
				k++;
			}
		data.SetVarianceMatrix(variance);
	}
	return data;
}

void SF::SerializeDataMsg(const DataMsg & dataMsg, void*& buf, int & length) {
	flatbuffers::FlatBufferBuilder fbb(1024);

	flatbuffers::Offset<flatbuffers::Vector<float>> fbb_value;
	float* v;
	if (dataMsg.HasValue()) {
		auto value = dataMsg.GetValue();
		size_t N = value.size();
		v = new float[N];
		for (unsigned int i = 0; i < N; i++)
			v[i] = (float)value[i];
		fbb_value = fbb.CreateVector<float>(v, N);
	}

	flatbuffers::Offset<flatbuffers::Vector<float>> fbb_variance;
	float* m;
	if (dataMsg.HasVariance()) {
		auto variance = dataMsg.GetVariance();
		unsigned int N = static_cast<unsigned int>(variance.rows());
		unsigned int K = static_cast<unsigned int>((N*(N + 1)) / 2);
		m = new float[K];
		int k = 0;
		for (unsigned int i = 0; i < N; i++)
			for (unsigned int j = i; j < N; j++) {
				m[k] = (float)variance(i, j);
				k++;
			}
		fbb_variance = fbb.CreateVector<float>(m, K);
	}

	DataMsgContentNameSpace::MsgBuilder msgBuilder(fbb);

	if (dataMsg.HasValue())
		msgBuilder.add_value_vector(fbb_value);

	if (dataMsg.HasVariance())
		msgBuilder.add_variance_matrix(fbb_variance);

	msgBuilder.add_timestamp_in_us(duration_since_epoch(dataMsg.GetTime()).count());

	auto msg = msgBuilder.Finish();
	fbb.Finish(msg);

	length = fbb.GetSize();

	auto buf_ = new unsigned char[length];
	for (size_t i = 0; i < length; i++)
		buf_[i] = fbb.GetBufferPointer()[i];
	buf = buf_;

	if (dataMsg.HasValue())
		delete v;
	if (dataMsg.HasVariance())
		delete m;
}
