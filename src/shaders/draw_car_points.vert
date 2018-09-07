#version 330 core

layout (location = 0) in vec4  in_vertex;

// materials.
uniform mat4 mvp;

out vec4 color;

void main()
{

  gl_Position = mvp * vec4(in_vertex.xyz, 1.0);

  {
    color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
  }
}
