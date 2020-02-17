// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_MSGCONTENT_DATAMSGCONTENTNAMESPACE_H_
#define FLATBUFFERS_GENERATED_MSGCONTENT_DATAMSGCONTENTNAMESPACE_H_

#include "flatbuffers/flatbuffers.h"

namespace DataMsgContentNameSpace {

struct Msg;

struct Msg FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_VALUE_VECTOR = 4,
    VT_VARIANCE_MATRIX = 6,
    VT_TIMESTAMP_IN_US = 8
  };
  const flatbuffers::Vector<float> *value_vector() const {
    return GetPointer<const flatbuffers::Vector<float> *>(VT_VALUE_VECTOR);
  }
  const flatbuffers::Vector<float> *variance_matrix() const {
    return GetPointer<const flatbuffers::Vector<float> *>(VT_VARIANCE_MATRIX);
  }
  int64_t timestamp_in_us() const {
    return GetField<int64_t>(VT_TIMESTAMP_IN_US, 0);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_VALUE_VECTOR) &&
           verifier.VerifyVector(value_vector()) &&
           VerifyOffset(verifier, VT_VARIANCE_MATRIX) &&
           verifier.VerifyVector(variance_matrix()) &&
           VerifyField<int64_t>(verifier, VT_TIMESTAMP_IN_US) &&
           verifier.EndTable();
  }
};

struct MsgBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_value_vector(flatbuffers::Offset<flatbuffers::Vector<float>> value_vector) {
    fbb_.AddOffset(Msg::VT_VALUE_VECTOR, value_vector);
  }
  void add_variance_matrix(flatbuffers::Offset<flatbuffers::Vector<float>> variance_matrix) {
    fbb_.AddOffset(Msg::VT_VARIANCE_MATRIX, variance_matrix);
  }
  void add_timestamp_in_us(int64_t timestamp_in_us) {
    fbb_.AddElement<int64_t>(Msg::VT_TIMESTAMP_IN_US, timestamp_in_us, 0);
  }
  explicit MsgBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  MsgBuilder &operator=(const MsgBuilder &);
  flatbuffers::Offset<Msg> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Msg>(end);
    return o;
  }
};

inline flatbuffers::Offset<Msg> CreateMsg(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<float>> value_vector = 0,
    flatbuffers::Offset<flatbuffers::Vector<float>> variance_matrix = 0,
    int64_t timestamp_in_us = 0) {
  MsgBuilder builder_(_fbb);
  builder_.add_timestamp_in_us(timestamp_in_us);
  builder_.add_variance_matrix(variance_matrix);
  builder_.add_value_vector(value_vector);
  return builder_.Finish();
}

inline flatbuffers::Offset<Msg> CreateMsgDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<float> *value_vector = nullptr,
    const std::vector<float> *variance_matrix = nullptr,
    int64_t timestamp_in_us = 0) {
  auto value_vector__ = value_vector ? _fbb.CreateVector<float>(*value_vector) : 0;
  auto variance_matrix__ = variance_matrix ? _fbb.CreateVector<float>(*variance_matrix) : 0;
  return DataMsgContentNameSpace::CreateMsg(
      _fbb,
      value_vector__,
      variance_matrix__,
      timestamp_in_us);
}

inline const DataMsgContentNameSpace::Msg *GetMsg(const void *buf) {
  return flatbuffers::GetRoot<DataMsgContentNameSpace::Msg>(buf);
}

inline const DataMsgContentNameSpace::Msg *GetSizePrefixedMsg(const void *buf) {
  return flatbuffers::GetSizePrefixedRoot<DataMsgContentNameSpace::Msg>(buf);
}

inline bool VerifyMsgBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<DataMsgContentNameSpace::Msg>(nullptr);
}

inline bool VerifySizePrefixedMsgBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<DataMsgContentNameSpace::Msg>(nullptr);
}

inline void FinishMsgBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<DataMsgContentNameSpace::Msg> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedMsgBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<DataMsgContentNameSpace::Msg> root) {
  fbb.FinishSizePrefixed(root);
}

}  // namespace DataMsgContentNameSpace

#endif  // FLATBUFFERS_GENERATED_MSGCONTENT_DATAMSGCONTENTNAMESPACE_H_
