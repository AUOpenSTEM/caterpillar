// based on servo_bracket-thingi-40590 (Creative Commons Attribution-ShareAlike)

// SG90 mini servo
servo_width				= 12.46;
servo_length				= 22.7;
servo_brace_thickness	= 2.5;

base_length				= servo_length+10;
base_width					= 10;	// can't be longer as it bumps into servo wiring otherwise
base_thickness			= 3;

// -----


//servo_bracket(true);		// 4x with foot hold for step segment
servo_bracket(false);		// 2x without (mirrored - relative to ones with foothold)
//step_segment();			// 4x step segment



module step_segment()
{
		rounding = 5;
		step_segment_width = servo_width + base_thickness + servo_width/2;

		hole_diameter = 2.5;

	rotate(a=[-90,,0])		// rotate into printable plane

		translate ([-base_thickness,25,-5])
			rotate ([90,0,0])
			union() {
				difference () {
					// bar
					minkowski () {
						 cube([step_segment_width - rounding * 2,base_length,base_thickness], center = true);
						 cylinder(r=rounding,h=1,$fn=50);
					}

					translate (v=[0,-15,0])
						cylinder(h = 30,d=hole_diameter, center = true, $fn=100);

					translate (v=[0,15,0])
						cylinder(h = 30,d=hole_diameter, center = true, $fn=100);

				}

				// connector
				translate ([0,-6.5 /* offset from middle */,5])
					difference () {
						union () {
							cube([step_segment_width,base_thickness,5], center = true);
							// extra bit in section that doesn't have the servo brace
							translate (v=[servo_width/2,servo_brace_thickness,0])
								cube([step_segment_width - servo_width,base_thickness,5], center = true);
						}

						translate (v=[-4.5,2.5,0])
							rotate(a=[90,0,0])
							cylinder(h = 30,d=hole_diameter, center = true, $fn=100);

						translate (v=[7.5,2.5,0])
							rotate(a=[90,0,0])
							cylinder(h = 30,d=hole_diameter, center = true, $fn=100);
					}
			}
}



module servo_bracket(with_foothold)
{
	rotate(a=[0,-90,0])								// rotate into printable plane
		mirror ([0,0,with_foothold ? 0 : 1])		// mirror depending on with/without foothold
			union () {
				// bar around which everything gets built
				cube ([base_width,base_length,base_thickness], center = true);

				// two arms to hold the servo
				translate ([-base_width/2,servo_length/2,base_thickness/2])
					arm(servo_width,base_width,base_thickness);

				mirror ([0,1,0])
					translate ([-base_width/2,servo_length/2,base_thickness/2])
					arm(servo_width,base_width,base_thickness);

				// leg to which horn of next servo is attached
				translate ([-base_width/2,-base_length/2,0])
					leg(base_width,base_length,base_thickness);

				// brace for long leg
				mirror ([0,0,1])
					translate (v=[-((base_width/2)-base_thickness),-(base_length/2 - 2), 0])
					rotate(a=[0,0,90])
					brace(servo_width,base_width,base_thickness);

				if (with_foothold == true) {
					// foot opposite one arm to which caterpillar foot bracket gets attached
					translate ([-base_width/2,servo_length/2,-(base_thickness/2)])
						foothold(servo_width/2,base_width,base_thickness);
				}
			}
}





module arm(servo_width,base_width,base_thickness)
{
	hole_diameter = 1.8;

	union () {
		difference () {
			translate ([0,0,0])
				cube( [base_thickness,5,servo_width]);	 

			// screw hole
			translate (v=[0,2.5,servo_width/2])
				rotate(a=[0,90,0])
				cylinder(h = 30,r=hole_diameter /2, center = true, $fn=100);
		}

		translate (v=[base_thickness,0,0])
			brace(servo_width,base_width-base_thickness,base_thickness);
	}
}



module foothold(servo_width,base_width,base_thickness)
{	
	mirror ([0,0,1])
		arm(servo_width,base_width,base_thickness);
}



/*
	 ___
	|   |
	|   |
	| o |	screw leg -> servo horn
	|   |
	| o |	screw leg -> servo horn
	| O |	through hole for screw servo horn -> axle
	|___|
*/
module leg(servo_width,base_length,base_thickness)
{

	hole_diameter = 2.5;
	screwhead_diameter = 4;
		
	rotate(a=[-90,0,0])
		union () {
			difference () {
				cube([servo_width,base_length, base_thickness], center = false);	//leg

				translate ([servo_width/2,base_length-(11.7/2),0])		// for screw into servo arm -> axle
					cylinder(h = 30,d=screwhead_diameter, center = true, $fn=100);

				translate ([servo_width/2,(base_length-(11.7/2))-5,0])
					cylinder(h = 30,d=hole_diameter, center = true, $fn=100);

				translate ([servo_width/2,((base_length-(11.7/2))-5)-9.5,0])
					cylinder(h = 30,d=hole_diameter, center = true, $fn=100);
			}

		}

}


/*
	|\
	| \
	|__\
*/
module brace (height, width, thickness)
{
	polyhedron (
		points = [[0, 0, 0], [0, 0, height], [width, 0, 0],	// front
					[0, thickness, 0], [0, thickness, height], [width, thickness, 0]], 	// back
		faces = [[0,1,2], [5,4,3],	// front, back
					[0,3,4,1], [1,4,5,2], [0,2,5,3] ]		// left, hypothenuse, bottom
		);
}


