#include <time.h>
#include "XrApp.h"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr const char* kLogTag = "QuestCapture";
constexpr float kRequestedRefreshHz = 90.0f;

int64_t MonotonicNs() {
    timespec time{};
    clock_gettime(CLOCK_MONOTONIC, &time);
    return int64_t{time.tv_sec} * 1000000000 + time.tv_nsec;
}

std::ostringstream JsonStream() {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(9);
    return stream;
}

std::string PoseFields(const XrPosef& pose, bool positionValid, bool orientationValid) {
    auto json = JsonStream();
    json << "\"p\":";
    if (positionValid) {
        json << '[' << pose.position.x << ',' << pose.position.y << ',' << pose.position.z << ']';
    } else {
        json << "null";
    }
    json << ",\"q\":";
    if (orientationValid) {
        json << '[' << pose.orientation.x << ',' << pose.orientation.y << ','
             << pose.orientation.z << ',' << pose.orientation.w << ']';
    } else {
        json << "null";
    }
    return json.str();
}

std::string PoseJson(const XrSpaceLocation& location, const char* active) {
    const auto flags = location.locationFlags;
    auto json = JsonStream();
    json << '{' << PoseFields(
                location.pose,
                (flags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0,
                (flags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0)
         << ",\"flags\":" << flags << ",\"active\":" << active << '}';
    return json.str();
}

class QuestCapture final : public OVRFW::XrApp {
  protected:
    std::unordered_map<XrPath, std::vector<XrActionSuggestedBinding>> GetSuggestedBindings(
        XrInstance instance) override {
        auto bindings = XrApp::GetSuggestedBindings(instance);
        XrPath simpleProfile = XR_NULL_PATH;
        if (!Check("xrStringToPath", xrStringToPath(
                instance, "/interaction_profiles/khr/simple_controller", &simpleProfile))) {
            ShouldExit = true;
            return {};
        }
        // The manifest permits controller-free launch, but this stream is for
        // Touch controllers. Do not bind the framework's generic hand fallback.
        bindings.erase(simpleProfile);
        return bindings;
    }

    std::vector<const char*> GetExtensions() override {
        auto extensions = XrApp::GetExtensions();
        extensions.push_back(XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME);
        extensions.push_back(XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);
        return extensions;
    }

    void GetInitialSceneUri(std::string& uri) const override {
        uri.clear();
    }

    bool AppInit(const xrJava* context) override {
        auto* env = context->Env;
        jclass activityClass = env->GetObjectClass(context->ActivityObject);
        jmethodID getter = env->GetMethodID(activityClass, "getSessionId", "()Ljava/lang/String;");
        auto id = static_cast<jstring>(env->CallObjectMethod(context->ActivityObject, getter));
        const char* chars = env->GetStringUTFChars(id, nullptr);
        sessionId_ = chars;
        env->ReleaseStringUTFChars(id, chars);
        env->DeleteLocalRef(id);
        env->DeleteLocalRef(activityClass);

        BackgroundColor = OVR::Vector4f(0, 0, 0, 1);
        FramebufferResolutionScaleFactor = 0.25f;
        return LoadFunction("xrConvertTimespecTimeToTimeKHR", convertTime_)
            && LoadFunction("xrRequestDisplayRefreshRateFB", requestRefresh_)
            && LoadFunction("xrGetDisplayRefreshRateFB", getRefresh_);
    }

    bool SessionInit() override {
        if (!Check("xrRequestDisplayRefreshRateFB", requestRefresh_(Session, kRequestedRefreshHz))
            || !Check("xrGetDisplayRefreshRateFB", getRefresh_(Session, &refreshHz_))) {
            ShouldExit = true;
            return false;
        }
        auto details = JsonStream();
        details << "{\"from_hz\":0,\"to_hz\":" << refreshHz_ << '}';
        Event("refresh_rate", details.str());
        return true;
    }

    // The framework already synchronizes its action sets immediately before Update.
    void Update(const OVRFW::ovrApplFrameIn&) override {
        timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);
        const int64_t queryNs = int64_t{now.tv_sec} * 1000000000 + now.tv_nsec;
        XrTime queryTime = 0;
        if (!Check("xrConvertTimespecTimeToTimeKHR", convertTime_(Instance, &now, &queryTime))) {
            ShouldExit = true;
            return;
        }
        while (!pendingOrigins_.empty() && pendingOrigins_.front().first <= queryTime) {
            origin_ = pendingOrigins_.front().second;
            pendingOrigins_.erase(pendingOrigins_.begin());
        }

        const bool leftActive = GripActive(LeftHandPath);
        const bool rightActive = GripActive(RightHandPath);
        const auto head = Locate(HeadSpace, queryTime);
        const auto left = Locate(LeftControllerGripSpace, queryTime);
        const auto right = Locate(RightControllerGripSpace, queryTime);

        auto json = JsonStream();
        json << "{\"v\":1,\"type\":\"frame\",\"session\":\"" << sessionId_
             << "\",\"seq\":" << sequence_++ << ",\"origin\":" << origin_
             << ",\"query_ns\":" << queryNs << ",\"xr_time\":" << queryTime
             << ",\"state\":" << state_ << ",\"refresh_hz\":" << refreshHz_
             << ",\"head\":" << PoseJson(head, "null")
             << ",\"left\":" << PoseJson(left, leftActive ? "true" : "false")
             << ",\"right\":" << PoseJson(right, rightActive ? "true" : "false")
             << ",\"send_ns\":" << MonotonicNs() << '}';
        __android_log_write(ANDROID_LOG_INFO, kLogTag, json.str().c_str());
    }

    void AppHandleEvent(XrEventDataBaseHeader* event) override {
        auto details = JsonStream();
        switch (event->type) {
            case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
                const auto& change = *reinterpret_cast<XrEventDataReferenceSpaceChangePending*>(event);
                if (change.referenceSpaceType != XR_REFERENCE_SPACE_TYPE_LOCAL) {
                    break;
                }
                const uint64_t nextOrigin = ++lastOrigin_;
                pendingOrigins_.emplace_back(change.changeTime, nextOrigin);
                std::sort(pendingOrigins_.begin(), pendingOrigins_.end());
                details << "{\"change_time\":" << change.changeTime
                        << ",\"origin\":" << nextOrigin
                        << ",\"pose_valid\":" << (change.poseValid ? "true" : "false")
                        << ",\"pose_in_previous_space\":{"
                        << PoseFields(change.poseInPreviousSpace, change.poseValid, change.poseValid)
                        << "}}";
                Event("reference_space_change", details.str());
                break;
            }
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                const auto& change = *reinterpret_cast<XrEventDataSessionStateChanged*>(event);
                state_ = change.state;
                details << "{\"state\":" << state_ << ",\"time\":" << change.time << '}';
                Event("session_state", details.str());
                break;
            }
            case XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB: {
                const auto& change = *reinterpret_cast<XrEventDataDisplayRefreshRateChangedFB*>(event);
                refreshHz_ = change.toDisplayRefreshRate;
                details << "{\"from_hz\":" << change.fromDisplayRefreshRate
                        << ",\"to_hz\":" << refreshHz_ << '}';
                Event("refresh_rate", details.str());
                break;
            }
            default:
                break;
        }
    }

  private:
    template <typename Function>
    bool LoadFunction(const char* name, Function& function) {
        return Check(name, xrGetInstanceProcAddr(
            Instance, name, reinterpret_cast<PFN_xrVoidFunction*>(&function)));
    }

    bool Check(const char* operation, XrResult result) {
        if (XR_SUCCEEDED(result)) {
            return true;
        }
        Event("error", "{\"operation\":\"" + std::string(operation)
            + "\",\"result\":" + std::to_string(result) + '}');
        return false;
    }

    bool GripActive(XrPath hand) {
        XrActionStateGetInfo info{XR_TYPE_ACTION_STATE_GET_INFO};
        info.action = GripPoseAction;
        info.subactionPath = hand;
        XrActionStatePose state{XR_TYPE_ACTION_STATE_POSE};
        return Check("xrGetActionStatePose", xrGetActionStatePose(Session, &info, &state))
            && state.isActive == XR_TRUE;
    }

    XrSpaceLocation Locate(XrSpace space, XrTime time) {
        XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
        if (!Check("xrLocateSpace", xrLocateSpace(space, LocalSpace, time, &location))) {
            location.locationFlags = 0;
        }
        return location;
    }

    void Event(const char* name, const std::string& details) const {
        auto json = JsonStream();
        json << "{\"v\":1,\"type\":\"event\",\"session\":\"" << sessionId_
             << "\",\"event\":\"" << name << "\",\"device_ns\":" << MonotonicNs()
             << ",\"details\":" << details << '}';
        __android_log_write(ANDROID_LOG_INFO, kLogTag, json.str().c_str());
    }

    std::string sessionId_;
    uint64_t sequence_ = 0;
    uint64_t origin_ = 0;
    uint64_t lastOrigin_ = 0;
    std::vector<std::pair<XrTime, uint64_t>> pendingOrigins_;
    XrSessionState state_ = XR_SESSION_STATE_UNKNOWN;
    float refreshHz_ = 0;
    PFN_xrConvertTimespecTimeToTimeKHR convertTime_ = nullptr;
    PFN_xrRequestDisplayRefreshRateFB requestRefresh_ = nullptr;
    PFN_xrGetDisplayRefreshRateFB getRefresh_ = nullptr;
};

}  // namespace

ENTRY_POINT(QuestCapture)
