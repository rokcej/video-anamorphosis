# Video Anamorphosis

The goal of this project is to project a distorted image onto a given surface, that looks perfect when viewed from a specific angle.

## Dependencies

| Dependency | Version | Description             |
|------------|--------:|-------------------------|
| GLFW       | 3.3     | OpenGL context creation |
| GLEW       | 2.1.0   | OpenGL extension loader |
| GLM        | 0.9.9.6 | OpenGL math library     |
| Kinect SDK | 2.0     | Windows Kinect API      |
| stb_image  | 2.23    | Image loading library   |
| FFmpeg     | 4.2.1   | Video loading library   |

## Controls

| Key          | Function                                |
|--------------|-----------------------------------------|
| R            | Toggle depth recording                  |
| V            | Switch between video and image mode     |
| D            | Toggle debug mode                       |
| Up, Down     | Rotate view                             |
| Left, Right  | Resize projection                       |
| Scroll wheel | Change projection radius                |
| W            | Toggle wireframe mode                   |
| M            | Toggle missing data approximation       |
| C            | Toggle calibration mode                 |
| K            | Switch between point mapping modes      |
| Space        | Toggle between live view and projection |
| Enter        | Apply projection to current position    |
| 1            | Move camera to projector view           |
| 2            | Move camera to observer view            |
| Esc          | Exit                                    |
