# hl_ur5_ik
Package to visualize ik of ur5 in Hololens2


## Unity Setup To Work with ROS package
1. Download and install [Unity Hub](https://unity3d.com/get-unity/download) and [Visual Studio Community](https://visualstudio.microsoft.com/).

![](/doc_images/unity_download.png)

![](/doc_images/vs_download.png)

2. Download the following [repo](https://data.4tu.nl/articles/software/Framework_for_the_publication_MirrorLabs_creating_similar_learning_environments_for_students_all_over_Europe_for_human-robot_coproduction/14186807).
3. Extract the repo zip in desired folder.
4. Open Unity Hub, click "ADD", find the MirrorLabs_HL2 folder and click "Open"
5. Download the desired version of unity by clicking the sign and following the "INSTALL" button. Make sure to also tick the UWP support.

![](/doc_images/version_install.png)

![](/doc_images/version_install_2.png)

![](/doc_images/version_install_3.png)

![](/doc_images/unity_install.png)

6. Open the project
7. In project open the scene called "ML_UniversalRobotic_ur5"

![](/doc_images/scene.png)

8. Save it as a new scene and name it the way you see fit.

![](/doc_images/copy_scene.png)

9. Create the sphere object in base_link object and locate in near the end effector of UR5.

![](/doc_images/control_sphere.png)

10. In RocConnectors/ur5 add script called "PoseStampedPublisher.cs" and assign the created sphre to the "Published Transform" field.

11. Open the script and modify 2 lines to look like in the following screenshot:

![](/doc_images/script_modif.png)

12. Change the topics for Joint State Publisher and Pose Stamped Publisher to the ones on the screenshot:

![](/doc_images/topic_names.png)


## Setting the project to run on HoloLens

Tom make the demo interactable in Hololens, we need to be able to move the sphere with our hands.

To do so you need to:

1. Add `NearInteractionGrabbable.cs` script as one of our sphere components.

![](/doc_images/HL_sphere.png)

2. Add `ManipulationHandler.cs` script as our sphere component.

![](/doc_images/HL_sphere2.png)

And choose the sphere as the host transform.

![](/doc_images/HL_sphere3.png)

Project is ready to be built for HoloLens2.

## Building the project for HoloLens

Before building< make sure, that in VisualStudio Installer you have installed the support for UWP development

![](/doc_images/UWP_support.png)

![](/doc_images/UWP_support2.png)

Now:

1. Go to `File > Build Settings` or ctrl+shift+b

2. Press `Add Open Scenes`. Choose UWP and ARM64 as architecture. Final result should look the following way. After that press `Switch Platform`

![](/doc_images/platform_switch.png)

3. Press `Build`

![](/doc_images/build_prep.png)

4. You will be asked to choose the folder for the project. Create the sepparate folder and name it as you see fit. In my case it is `App`.

![](/doc_images/building.png)

5. After build is finished, go to the created folder and open the file named `ML_M1013.sln`

![](/doc_images/solution_opening.png)

6. In Visual Studio asks you to install some components, agree to it.

![](/doc_images/sudden_sdk_installation.png)

7. In opened Visual studio solution go to `Project > Publish > Create App Package...`

![](/doc_images/app_creation.png)

8. Click next until you get to choose version, path and architecture. Remember the path and ignore version. For architecture choose `ARM64` and release configuretion.

![](/doc_images/app_creation2.png)

![](/doc_images/app_creation3.png)

![](/doc_images/app_creation4.png)

9. Press `Create`.

## Deployment to HoloLens(TODO)