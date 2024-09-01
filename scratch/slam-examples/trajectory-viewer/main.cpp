#include <pangolin/pangolin.h>
#include <fstream>
#include <sstream>

// Function to read camera trajectory from file
void ReadCameraTrajectory(const std::string& filename, std::vector<Eigen::Vector3d>& trajectory) {
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double timestamp, x, y, z;
        iss >> timestamp >> x >> y >> z;
        trajectory.emplace_back(x, y, z);
    }
}


int main() {
    // Create a Pangolin window
    pangolin::CreateWindowAndBind("ORB_SLAM3 Visualization", 1024, 768);

// Add this line after creating the window
glewInit();

    // Set up the view
    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                      pangolin::ModelViewLookAt(-0.5, 0.5, -1, 0, 0, 0, pangolin::AxisY));

    // Create a Pangolin 'handler' for interactive viewing
    pangolin::Handler3D handler(s_cam);

    // Create a View
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
            .SetHandler(&handler);

    // Read camera and keyframe trajectories from file
    std::vector<Eigen::Vector3d> cameraTrajectory, keyframeTrajectory;
    ReadCameraTrajectory("../CameraTrajectory.txt", cameraTrajectory);
    ReadCameraTrajectory("../KeyFrameTrajectory.txt", keyframeTrajectory);
	
	/**/
	for (const auto& pose : cameraTrajectory) {
		std::cout << "Camera Pose: " << pose.transpose() << std::endl;
	}

	for (const auto& pose : keyframeTrajectory) {
		std::cout << "Keyframe Pose: " << pose.transpose() << std::endl;
	}
	/**/

    // Main loop
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw camera trajectory
        glColor3f(1.0f, 0.0f, 0.0f);  // Red color for the camera trajectory
        glBegin(GL_LINE_STRIP);
        for (const auto& pose : cameraTrajectory) {
            glVertex3d(pose.x(), pose.y(), pose.z());
        }
        glEnd();

        // Draw keyframe trajectory
        glColor3f(0.0f, 0.0f, 1.0f);  // Blue color for the keyframe trajectory
        glBegin(GL_LINE_STRIP);
        for (const auto& pose : keyframeTrajectory) {
            glVertex3d(pose.x(), pose.y(), pose.z());
        }
        glEnd();

        pangolin::FinishFrame();

		GLenum error = glGetError();
		if (error != GL_NO_ERROR) {
			std::cerr << "OpenGL Error: " << error << std::endl;
		}

    }

    return 0;
}

