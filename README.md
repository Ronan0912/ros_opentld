This is a ROS version of the OpenTLD tracker.

OpenTLD is a C++ implementation of TLD Predator (Tracking. Learning and Detection) implemented by the AIT (Austrian Institute of Technology) that was originally published in MATLAB by Zdenek Kalal. OpenTLD is used for tracking objects in video streams. It doesn't need any training data and is also able to load predefined models (http://gnebehay.github.com/OpenTLD/).

The ROS implementation consists of two nodes : the tracker node which use the opentld library and an interface node that allow you to select a bounding box, start stop the tracking, start and stop the learning, import or export a model, clear the background and change the tracker's method.

## Keyboard shortcuts for the interface (same as OpenTLD)

* `q` quit
* `b` remember current frame as background model / clear background
* `c` stop/resume tracking
* `l` toggle learning
* `a` toggle alternating mode (if true, detector is switched off when tracker is available)
* `e` export model
* `i` import model
* `r` clear model

In the two launch files, you can configure the input video stream. In the tracker launch file, you can configure the bounding box source, the default bounding box if there is one, the model that you may want to load and its path, the automatic face detection by the OpenCV cascade classifier and some others parameters.

Like OpenTLD, ROS_OpenTLD is published under the terms of the GNU General Public License.

IntRoLab
http://introlab.3it.usherbrooke.ca
Université de Sherbrooke, Québec, Canada
