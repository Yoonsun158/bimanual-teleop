# Third-party sources

The Quest APK uses Meta's native OpenXR sample framework as a build dependency. The application-specific acquisition and JSON protocol code is maintained in this project.

| Component | Pinned version / source | License |
| --- | --- | --- |
| Meta OpenXR SDK, SampleXrFramework and bundled dependencies | [v85, commit bbed2f20e38a5df7113630771c83cb8279e4fc26](https://github.com/meta-quest/Meta-OpenXR-SDK/tree/bbed2f20e38a5df7113630771c83cb8279e4fc26) | Meta/Oculus SDK license and the individual notices distributed with its first-party and third-party components; retain the SDK's supplied notices when redistributing those components. |
| Khronos OpenXR Android loader | `org.khronos.openxr:openxr_loader_for_android:1.1.53` | [OpenXR SDK license](https://github.com/KhronosGroup/OpenXR-SDK/blob/release-1.1.53/LICENSE) |
| Gradle wrapper | [Gradle 8.5.0](https://github.com/gradle/gradle/tree/v8.5.0) | [Apache License 2.0](https://github.com/gradle/gradle/blob/v8.5.0/LICENSE) |

The original Oculus Reader author maintains [jborbik/oculus_reader](https://github.com/jborbik/oculus_reader/tree/9689484d319c4798e54d59509b192436647b7427). Its APK-plus-Python organization, USB ADB transport and native SDK build configuration informed this module. Its controller UI, haptics demo, head-relative pose serialization and Python reader are not incorporated.

The Meta SDK archive is downloaded at build time with SHA-256 `0edf48d28b2e87e826d398d839d9cbeb3d0386b0a29a1b3bbfabda4d3e2255bf`. The SDK's original license notice, bundled third-party notices, KTX license directory, and OpenXR loader AAR's `META-INF/LICENSE` are copied verbatim to `src/main/assets/notices` and included in the APK. The SDK's own license file points to Meta's online license agreement; it is preserved as supplied.

Gradle's official wrapper JAR and launch scripts are included; its distribution license is preserved in `gradle/LICENSE.txt`, and the Gradle 8.5 distribution is checksum-pinned in `gradle-wrapper.properties`.
