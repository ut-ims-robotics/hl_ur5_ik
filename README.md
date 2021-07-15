# hl_ur5_ik
Package to visualize ik of ur5 in Hololens2


## Unity Setup
1. Download and install [Unity Hub](https://unity3d.com/get-unity/download) and [Visual Studio Community](https://visualstudio.microsoft.com/).

![](/doc_images/unity_download.png)

![](/doc_images/vs_download.png)

2. Download the following [repo (link will be soon)]().
3. Extract the repo zip in desired folder.
4. Open Unity Hub, click "ADD", find the MirrorLabs_HL2 folder and click "Open"
5. Download the desired version of unity by clicking the sign and following the "INSTALL" button.
6. Open the project
7. In project open the scene called "ML_UniversalRobotic_ur5"
8. Save it as a new scene and name it the way you see fit.

![](/doc_images/copy_scene.png)

9. Create the sphere object in base_link object and locate in near the end effector of UR5.

![](/doc_images/control_sphere.png)

10. In RocConnectors/ur5 add script called "PoseStampedPublisher.cs" and assign the created sphre to the "Published Transform" field.

11. Open the script and modify 2 line to look like in the following screenshot:

![](/doc_images/script_modif.png)

12. Change the topics for Joint State Publisher and Pose Stamped Publisher to the ones on the screenshot:

![](/doc_images/topic_names.png)


## Setting the project to run on HoloLens(TODO)
