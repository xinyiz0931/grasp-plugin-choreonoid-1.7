function main() {
   return union(
     cylinder({d:2.5, h:0.5, center: true}).translate([0, 0, -0.5/2]),
     cylinder({d:1.5, h:8, center: true}).translate([0, 0, -8/2])
   ).setColor([0.7,0.7,0.7]).scale(1/100);
}
