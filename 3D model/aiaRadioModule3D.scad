$fn=50;

use <grille.scad>

epsParoie = 1.5;
epsParoie2 = 2*epsParoie;


//piece1();

piece2();



module piece1() {       
    color("silver")difference(){
        minkowski() {
            bottomBox();
            sphere(epsParoie);
        }
        bottomBox();
        translate([-3,-3,4]) cube([65,65,6]);
    }
    difference(){
        translate([0,0,4-epsParoie]) cube([58,58,epsParoie2]);
        translate([epsParoie,epsParoie,3-epsParoie]) cube([58-epsParoie2,58-epsParoie2,epsParoie2+2]);
        translate([-3,-3,2]) cube([19,30,5]);
    }
}
module bottomBox(){

            translate([0,0,0]) cube([58,58,7]);  

}

module piece2() {       
    color("silver")difference(){
        minkowski() {
            Box();
            sphere(epsParoie);
        }
        Box();
        translate([-3,-3,-2]) cube([65,65,6]);
        translate([6,-3,3]) cube([8,5,5]);
        translate([-3,14,3]) cube([8,10,5]);
        translate([29,29,20]) cylinder(d=49,h=10); // grille
    }
    translate([9,9,26.1]) grill();
}
module Box(){
    difference(){
    union(){
            translate([0,0,0]) cube([58,58,26]);  

        }
        
    }
}