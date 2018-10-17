#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in uint  in_label;

uniform mat4 pose;
uniform uint scan;

out POINT
{
  vec4 point;
  uint label;
} vs_out;


void main()
{
  vec4 v_global = pose * vec4(in_vertex, 1.0);
  switch(in_label){
	  case 0:    //unlabeled
	  case 1:    //outlier
	  case 13:   //bus
	  case 16:   //on-rails
	  case 52:   //other-structure
	  case 90:   //bench
	  case 91:   //trash can
	  case 99:   //other-object
	  case 256:  //moving-on-rails
	  case 257:  //moving-bus

	  case 41:   //decepticon
	  case 46:   //giant-gummy-lizzard
	  case 76:   //floating-eyes
	  case 251:  //magic-portal
	  case 260:  //gummy-bear
	    vs_out.label = 0;    //   -> unlabeled
	    break;

	  case 10:   //car
	  case 252:  //moving car
	    vs_out.label = 1;    //   -> car
	    break;
	  case 11:   //bicycle
	    vs_out.label = 2;    //   -> bicycle
	    break;
	  case 15:   //motorcycle
	    vs_out.label = 3;    //   -> motorcycle
	    break;
	  case 18:   // truck
	  case 258:  // moving-truck
	    vs_out.label = 4;    //   -> truck
	    break;
	  case 20:   // other-vehicle
	  case 259:  // moving-other
	    vs_out.label = 5;    //   -> other vehicle
	    break;
	  case 30:   // person
	  case 254:  // moving-person
	    vs_out.label = 6 ;   //   -> person
	    break;
	  case 31:   // bicyclist
	  case 253:  // moving-bicyclist
	    vs_out.label = 7 ;   //   -> bicyclist
	    break;
	  case 32:   // motorcyclist
	  case 255:  // moving-motorcyclist
	    vs_out.label = 8 ;   //   -> motorcyclist
	    break;
	  case 40:   // road
	  case 60:   // lane-marking
	    vs_out.label = 9 ;   //   -> road
	    break;
	  case 44:   // parking
	    vs_out.label = 10;   //   -> parking
	    break;
	  case 48:   // sidewalk
	    vs_out.label = 11;   //   -> sidewalk
	    break;
	  case 49:   // other-ground
	    vs_out.label = 12;   //   -> other-ground
	    break;
	  case 50:   // building
	    vs_out.label = 13;   //   -> building
	    break;
	  case 51:   // fence
	    vs_out.label = 14;   //   -> fence
	    break;
	  case 70:   // vegetation
	    vs_out.label = 15;   //   -> vegetation
	    break;
	  case 71:   // trunk
	    vs_out.label = 16;   //   -> trunk
	    break;
	  case 72:   // terrain
	    vs_out.label = 17;   //   -> terrain
	    break;
	  case 80:   // pole
	    vs_out.label = 18;   //   -> pole
	    break;
	  case 81:   // traffic-sign
	    vs_out.label = 19;   //   -> traffic-sign
	    break;
	  default:
	    std::cout<<"Undefined behavior for label " + std::to_string(lb_)<<" in Seq "<<number<<", Scan "<<i<<"."<<std::endl;
	    throw "Undefined behavior for label " + std::to_string(lb_);

	}

}