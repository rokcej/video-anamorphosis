# Video Anamorphosis

The goal of this project is to project a distorted (anamorphic) image onto a given surface, that looks recognizable when viewed from a specific angle.

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

| Key          | Function                                             |
|--------------|------------------------------------------------------|
| R            | Toggle depth recording                               |
| V            | Switch between video and image mode                  |
| D            | Toggle debug mode                                    |
| O            | Project colour data back onto objects (calibration)  |
| Up, Down     | [Debug mode] Rotate view horizontally                |
| Left, Right  | [Debug mode] Move view vertically                    |
| Plus, Minus  | [Debug mode] Resize projection                       |
| Scroll wheel | [Debug mode] Change projection radius                |
| W            | [Debug mode] Toggle wireframe mode                   |
| M            | [Debug mode] Toggle missing data approximation       |
| C            | [Debug mode] Toggle calibration mode                 |
| K            | [Debug mode] Switch between point mapping modes      |
| Space        | [Debug mode] Toggle between live view and projection |
| Enter        | [Debug mode] Apply projection to current position    |
| 1            | [Debug mode] Move camera to projector view           |
| 2            | [Debug mode] Move camera to observer view            |
| Esc          | Exit                                                 |
