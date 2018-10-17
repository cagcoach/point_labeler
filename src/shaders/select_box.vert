#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in uint  in_label;
layout (location = 2) in uint  in_visible;

uniform mat4 centerpose_inverse;
uniform vec4 corner;


uniform bool removeGround;
uniform float groundThreshold;

uniform sampler2D heightMap;

uniform vec2 tilePos;
uniform float tileSize;

uniform bool planeRemoval;
uniform int planeDimension;
uniform float planeThreshold;
uniform float planeDirection;

out uint out_inbox;
void main()
{


  vec4 v_global = vec4(in_vertex.xyz, 1.0);
  vec2 v = v_global.xy - tilePos;
  bool visible = (in_visible > uint(0)) && (!removeGround || v_global.z > texture(heightMap, v / tileSize + 0.5).r + groundThreshold); 

  if(planeRemoval) visible = visible && (planeDirection * (in_vertex[planeDimension] - planeThreshold)  < 0);
  

  vec4 point = centerpose_inverse * v_global;

  if(visible && abs(point.x) <= corner.x && abs(point.y) <= corner.y && abs(point.z) <= corner.z ){
    out_inbox = 1u;
  }else{
    out_inbox = 0u;
  }
}