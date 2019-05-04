#pragma once
#include <vector>
#include <functional>

inline unsigned int getUID() {
	static unsigned int ID = 0;
	return ID++;
}

template <class ValueType,class CallType>
class CallbackHandler {
	unsigned int iID;

	typedef std::function<void(ValueType value, CallType type)> Callback;

	struct CallInfo {
		Callback callback;
		unsigned int ownerID;
		CallInfo(Callback call, unsigned int ID) : callback(call), ownerID(ID) {};
	};

	typedef std::vector<CallInfo> CallbackVector;

	CallbackVector vCallback;

public:
	unsigned int getID() const { return iID; }

	void AddCallback(Callback callback, unsigned int ownerID);

	void DeleteCallback(unsigned int ownerID);

	CallbackHandler();

	~CallbackHandler() {}

protected:
	void Call(ValueType value, CallType type) const;
};

template<class ValueType, class CallType>
void CallbackHandler<ValueType, CallType>::AddCallback(Callback callback, unsigned int ownerID) {
	for (unsigned int i = 0; i < vCallback.size(); i++)
		if (vCallback[i].ownerID == ownerID) {
			std::cout << "Callback tried to be added more times (System.h)\n";
			return;
		}
	vCallback.push_back(CallInfo(callback, ownerID));
}

template<class ValueType, class CallType>
void CallbackHandler<ValueType, CallType>::DeleteCallback(unsigned int ownerID) {
	for (unsigned int i = 0; i < vCallback.size(); i++)
		if (vCallback[i].ownerID == ownerID) {
			vCallback.erase(vCallback.begin() + i);
			return;
		}
	std::cout << "Not contained callback tried to be deleted (System.h)\n";
}

template<class ValueType, class CallType>
CallbackHandler<ValueType, CallType>::CallbackHandler() : vCallback(CallbackVector()) {
	iID = getUID();
}

template<class ValueType, class CallType>
void CallbackHandler<ValueType, CallType>::Call(ValueType value, CallType type) const {
	// call the registered callbacks
	for (unsigned int i = 0; i < vCallback.size(); i++)
		vCallback[i].callback(value, type);
}
