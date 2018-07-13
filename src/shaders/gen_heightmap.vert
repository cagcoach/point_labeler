#version 330 core

layout (location = 0) in vec4 in_vertex;
layout (location = 1) in uint in_label;

// uniform mat4 pose;

uniform vec2 tilePos;
uniform float tileSize;

uniform float minHeight;
uniform float maxHeight;

out float height;

void main()
{
  float range = length(in_vertex.xyz);
  

  vec2 v = vec4(in_vertex.xyz, 1.0).xy - tilePos;
  // bool visible =  (abs(v.x) < 0.5 * tileSize && abs(v.y) < 0.5 * tileSize);
  
  gl_Position = vec4(2.0 * v/tileSize, (in_vertex.z-minHeight) / (maxHeight-minHeight), 1);
  
  
  // ignore outliers.
  if(in_label == 1) gl_Position = vec(-10, -10, -10, 1);
  height = in_vertex.z;
}