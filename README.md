# Autonomous-Mobile-Robot
The objective was to perform autonomous navigation of a directed maze using a turtlebot.

It initially involved working with OpenCV libraries to detect objects, followed by implementing PID controller to track the object at a particular distance. Next, I used odometry and the previously implemented PID control to take the bot from point A to point B. Once that was done, I implemented a state machine that could switch between obstacle avoidance and goto goal behavior. The next step was to use Navstack to navigate the robot in a localized environment. Finally, built a knn based ML model to detect directions signs. 
All the above tasks culminated in the final demonstrations where the turtlebot was able to navigate out of directed maze successfully.

Link to demonstration video: https://www.youtube.com/watch?v=RPwdeLt1w-s&ab_channel=PrajwalBharadwaj


