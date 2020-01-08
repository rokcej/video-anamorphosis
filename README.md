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

## Controls

| Key         | Function                                |
|-------------|-----------------------------------------|
| Left, Right | Rotate view                             |
| Up, Down    | Resize projection                       |
| Space       | Toggle between live view and projection |
| Enter       | Apply projection to current position    |
| 1           | Move camera to projector view           |
| 2           | Move camera to observer view            |
| Esc         | Exit                                    |
