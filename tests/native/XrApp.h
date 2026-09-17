// Minimal test double for exercising our acquisition code without an XR runtime.
// The Android build separately compiles against the real pinned OpenXR headers.
#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

using XrTime = int64_t;
using XrResult = int;
using XrSpace = int;
using XrPath = int;
using XrInstance = int;
using XrSession = int;
using XrSessionState = int;
using XrSpaceLocationFlags = uint64_t;
constexpr int XR_TRUE = 1;
constexpr int XR_NULL_PATH = 0;
constexpr int XR_SESSION_STATE_UNKNOWN = 0;
constexpr int XR_TYPE_ACTION_STATE_GET_INFO = 1, XR_TYPE_ACTION_STATE_POSE = 2;
constexpr int XR_TYPE_SPACE_LOCATION = 3;
constexpr int XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING = 4;
constexpr int XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED = 5;
constexpr int XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB = 6;
constexpr int XR_REFERENCE_SPACE_TYPE_LOCAL = 1;
constexpr int XR_SPACE_LOCATION_ORIENTATION_VALID_BIT = 1;
constexpr int XR_SPACE_LOCATION_POSITION_VALID_BIT = 2;
constexpr int ANDROID_LOG_INFO = 4;
constexpr auto XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME = "convert-time";
constexpr auto XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME = "refresh-rate";
#define XR_SUCCEEDED(result) ((result) >= 0)

struct XrVector3f { float x = 0, y = 0, z = 0; };
struct XrQuaternionf { float x = 0, y = 0, z = 0, w = 0; };
struct XrPosef { XrQuaternionf orientation; XrVector3f position; };
struct XrSpaceLocation { int type; XrSpaceLocationFlags locationFlags = 0; XrPosef pose; };
struct XrActionStateGetInfo { int type; int action = 0; XrPath subactionPath = 0; };
struct XrActionStatePose { int type; int isActive = 0; };
struct XrActionSuggestedBinding { int action; XrPath binding; };
struct XrEventDataBaseHeader { int type; };
struct XrEventDataReferenceSpaceChangePending {
    int type = XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING;
    int referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    XrTime changeTime = 0;
    bool poseValid = false;
    XrPosef poseInPreviousSpace;
};
struct XrEventDataSessionStateChanged {
    int type = XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED;
    XrSessionState state = 5;
    XrTime time = 950;
};
struct XrEventDataDisplayRefreshRateChangedFB {
    int type = XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB;
    float fromDisplayRefreshRate = 90, toDisplayRefreshRate = 72;
};

using PFN_xrVoidFunction = void (*)();
using PFN_xrConvertTimespecTimeToTimeKHR = XrResult (*)(XrInstance, const timespec*, XrTime*);
using PFN_xrRequestDisplayRefreshRateFB = XrResult (*)(XrSession, float);
using PFN_xrGetDisplayRefreshRateFB = XrResult (*)(XrSession, float*);

inline XrTime testTime = 1000;
inline int queryCount = 0;
inline bool failRefresh = false;
inline XrResult xrStringToPath(XrInstance, const char* path, XrPath* result) {
    assert(std::strcmp(path, "/interaction_profiles/khr/simple_controller") == 0);
    *result = 20;
    return 0;
}
inline XrResult ConvertTime(XrInstance, const timespec*, XrTime* time) {
    *time = testTime;
    return 0;
}
inline XrResult RequestRefresh(XrSession, float hz) {
    assert(hz == 90.0f);
    return failRefresh ? -1 : 0;
}
inline XrResult GetRefresh(XrSession, float* hz) { *hz = 90; return 0; }
inline XrResult xrGetInstanceProcAddr(XrInstance, const char* name, PFN_xrVoidFunction* function) {
    if (std::strcmp(name, "xrConvertTimespecTimeToTimeKHR") == 0) {
        *function = reinterpret_cast<PFN_xrVoidFunction>(ConvertTime);
    } else if (std::strcmp(name, "xrRequestDisplayRefreshRateFB") == 0) {
        *function = reinterpret_cast<PFN_xrVoidFunction>(RequestRefresh);
    } else {
        assert(std::strcmp(name, "xrGetDisplayRefreshRateFB") == 0);
        *function = reinterpret_cast<PFN_xrVoidFunction>(GetRefresh);
    }
    return 0;
}
inline XrResult xrGetActionStatePose(XrSession, const XrActionStateGetInfo* info, XrActionStatePose* state) {
    assert(info->action == 10);
    state->isActive = info->subactionPath == 11;
    return 0;
}
inline XrResult xrLocateSpace(XrSpace space, XrSpace base, XrTime time, XrSpaceLocation* location) {
    assert(base == 2 && time == testTime);
    assert(space == (queryCount % 3 == 0 ? 1 : queryCount % 3 == 1 ? 3 : 4));
    ++queryCount;
    location->locationFlags = space == 1 ? 15 : space == 3 ? 5 : 10;
    location->pose.position = {float(space), 2, 3};
    location->pose.orientation = {0, 0, 0, 1};
    return 0;
}
inline void __android_log_write(int, const char* tag, const char* json) {
    assert(std::strcmp(tag, "QuestCapture") == 0);
    std::cout << json << '\n';
}

using jclass = void*;
using jmethodID = void*;
using jstring = const char*;
struct JNIEnv {
    jclass GetObjectClass(void*) { return nullptr; }
    jmethodID GetMethodID(jclass, const char*, const char*) { return nullptr; }
    jstring CallObjectMethod(void*, jmethodID) { return "0123456789abcdef0123456789abcdef"; }
    const char* GetStringUTFChars(jstring id, void*) { return id; }
    void ReleaseStringUTFChars(jstring, const char*) {}
    void DeleteLocalRef(const void*) {}
};
struct xrJava { JNIEnv* Env; void* ActivityObject = nullptr; };
namespace OVR { struct Vector4f { Vector4f(float, float, float, float) {} }; }

namespace OVRFW {
struct ovrApplFrameIn {};
class XrApp {
  public:
    virtual ~XrApp() = default;
    void RunTest();
  protected:
    virtual std::vector<const char*> GetExtensions() { return {}; }
    virtual std::unordered_map<XrPath, std::vector<XrActionSuggestedBinding>> GetSuggestedBindings(
        XrInstance) { return {{20, {{10, 30}}}, {21, {{10, 31}, {10, 32}}}}; }
    virtual void GetInitialSceneUri(std::string&) const {}
    virtual bool AppInit(const xrJava*) { return true; }
    virtual bool SessionInit() { return true; }
    virtual void Update(const ovrApplFrameIn&) {}
    virtual void AppHandleEvent(XrEventDataBaseHeader*) {}
    XrSpace HeadSpace = 1, LocalSpace = 2, LeftControllerGripSpace = 3, RightControllerGripSpace = 4;
    XrInstance Instance = 5;
    XrSession Session = 6;
    int GripPoseAction = 10;
    XrPath LeftHandPath = 11, RightHandPath = 12;
    bool ShouldExit = false;
    OVR::Vector4f BackgroundColor{0, 0, 0, 0};
    float FramebufferResolutionScaleFactor = 1;
};

inline void XrApp::RunTest() {
    JNIEnv env;
    xrJava java{&env};
    assert(AppInit(&java));
    const auto bindings = GetSuggestedBindings(Instance);
    assert(bindings.size() == 1 && bindings.count(21) == 1);
    assert(bindings.at(21).size() == 2);  // retain both Touch controller bindings
    assert(SessionInit());
    XrEventDataSessionStateChanged state;
    AppHandleEvent(reinterpret_cast<XrEventDataBaseHeader*>(&state));
    XrEventDataReferenceSpaceChangePending change;
    change.changeTime = 1200;
    AppHandleEvent(reinterpret_cast<XrEventDataBaseHeader*>(&change));
    Update({});
    testTime = 1200;
    Update({});
    assert(queryCount == 6);
    XrEventDataDisplayRefreshRateChangedFB refresh;
    AppHandleEvent(reinterpret_cast<XrEventDataBaseHeader*>(&refresh));
    testTime = 1300;
    Update({});
    failRefresh = true;
    assert(!SessionInit() && ShouldExit);
}
}  // namespace OVRFW

#define ENTRY_POINT(appClass) int main() { appClass app; app.RunTest(); }
