#pragma once
#include <vector>
#include <functional>

inline unsigned int getUID() {
	static unsigned int ID = 0;
	return ID++;
}

template <class CallbackData>
class CallbackHandler {
public:
	typedef std::function<void(const CallbackData& data)> Callback;

	unsigned int getID() const { return iID; }

	void AddCallback(Callback callback, unsigned int ownerID);

	void DeleteCallback(unsigned int ownerID);

	CallbackHandler();

	~CallbackHandler() {}

protected:
	void Call(const CallbackData& data) const;

private:
	unsigned int iID;

	struct CallInfo {
		Callback callback;
		unsigned int ownerID;
		CallInfo(Callback call, unsigned int ID) : callback(call), ownerID(ID) {};
	};

	typedef std::vector<CallInfo> CallbackVector;

	CallbackVector vCallback;
};

template<class CallbackData>
void CallbackHandler<CallbackData>::AddCallback(Callback callback, unsigned int ownerID) {
	for (unsigned int i = 0; i < vCallback.size(); i++)
		if (vCallback[i].ownerID == ownerID) {
			std::cout << "Callback tried to be added more times (System.h)\n";
			return;
		}
	vCallback.push_back(CallInfo(callback, ownerID));
}

template<class CallbackData>
void CallbackHandler<CallbackData>::DeleteCallback(unsigned int ownerID) {
	for (unsigned int i = 0; i < vCallback.size(); i++)
		if (vCallback[i].ownerID == ownerID) {
			vCallback.erase(vCallback.begin() + i);
			return;
		}
	std::cout << "Not contained callback tried to be deleted (System.h)\n";
}

template<class CallbackData>
CallbackHandler<CallbackData>::CallbackHandler() : vCallback(CallbackVector()) {
	iID = getUID();
}

template<class CallbackData>
void CallbackHandler<CallbackData>::Call(const CallbackData& data) const {
	// call the registered callbacks
	for (unsigned int i = 0; i < vCallback.size(); i++)
		vCallback[i].callback(data);
}
