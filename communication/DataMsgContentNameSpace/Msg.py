# automatically generated by the FlatBuffers compiler, do not modify

# namespace: DataMsgContentNameSpace

import flatbuffers

class Msg(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsMsg(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = Msg()
        x.Init(buf, n + offset)
        return x

    # Msg
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Msg
    def ValueVector(self, j):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            a = self._tab.Vector(o)
            return self._tab.Get(flatbuffers.number_types.Float32Flags, a + flatbuffers.number_types.UOffsetTFlags.py_type(j * 4))
        return 0

    # Msg
    def ValueVectorAsNumpy(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.GetVectorAsNumpy(flatbuffers.number_types.Float32Flags, o)
        return 0

    # Msg
    def ValueVectorLength(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.VectorLen(o)
        return 0

    # Msg
    def VarianceMatrix(self, j):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            a = self._tab.Vector(o)
            return self._tab.Get(flatbuffers.number_types.Float32Flags, a + flatbuffers.number_types.UOffsetTFlags.py_type(j * 4))
        return 0

    # Msg
    def VarianceMatrixAsNumpy(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.GetVectorAsNumpy(flatbuffers.number_types.Float32Flags, o)
        return 0

    # Msg
    def VarianceMatrixLength(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.VectorLen(o)
        return 0

    # Msg
    def TimestampInUs(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Int64Flags, o + self._tab.Pos)
        return 0

def MsgStart(builder): builder.StartObject(3)
def MsgAddValueVector(builder, valueVector): builder.PrependUOffsetTRelativeSlot(0, flatbuffers.number_types.UOffsetTFlags.py_type(valueVector), 0)
def MsgStartValueVectorVector(builder, numElems): return builder.StartVector(4, numElems, 4)
def MsgAddVarianceMatrix(builder, varianceMatrix): builder.PrependUOffsetTRelativeSlot(1, flatbuffers.number_types.UOffsetTFlags.py_type(varianceMatrix), 0)
def MsgStartVarianceMatrixVector(builder, numElems): return builder.StartVector(4, numElems, 4)
def MsgAddTimestampInUs(builder, timestampInUs): builder.PrependInt64Slot(2, timestampInUs, 0)
def MsgEnd(builder): return builder.EndObject()
