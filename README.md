# 16-423FinalProjet_PoolMaster

This is 16-423 Mobile Computer Vision final project at CMU.
You can see the working demo for this app here.
https://www.youtube.com/watch?v=roLgw40vlyU

When the application starts, user can select four corners by dragging the colors to the four corners on the billiard table. After selecting four corners press "Convex". Then, place the billiard balls on the table.
After placing the balls place, tap Predict to enter the prediction mode.
In prediction mode, tap each of three buttons on top, "Pick Cue", "Pick Target", "Pick Goal", as you select your cue ball, target ball, and goal destination respectively. After three positions are selected, the app shows the recommended position to align your cue. Tap Align! to align your cue in real world.
Tap "Start Tracking" button to start tracking your balls on the table. After the tracking finished, tap "Birdview" to see the trajectory on the birdview of the table.

From the project checkpoint, we have made significant improvements in the application. We cleaned up our UI so that flow of the application is clearer. We implemented ball tracking and birdview display of the trajecotires. We increased the performance of the application so that it runs at 30 FPS for 640x480 resolution and 24 FPS for 1280x720.