/*----------------------------------------------
/ Spherical coordinates to XYZ
/ Assuming Ascii inputs.
/ Order of cells: range, azimuth, polar, intensity
/----------------------------------------------*/

String inputFilename = "OL1.CSV";
String outputFilename = "OL1xyz.CSV";

PrintWriter myOutput; 

float range;
float azimuth;
float elevation;
float intensity;

float x;
float y;
float z;
float i;

long debugCount = 0;

Table inputTable;

//Table outputTable = new Table();
//outputTable.addColumn("x");
//outputTable.addColumn("y");
//outputTable.addColumn("z");
//outputTable.addColumn("i");

//inputTable = loadTable(inputFilename+".csv","header");  //our file has a header
//inputTable = loadTable("OL1.CSV", "header");  //our file has a header
inputTable = loadTable(inputFilename, "header");  //our file has a header
println("loaded " + inputFilename);
println(inputTable.getRowCount() + " total rows in table");

//myOutput = createWriter(outputFilename+".CSV");
//myOutput = createWriter("OL1xyz.csv");
myOutput = createWriter(outputFilename);
myOutput.println("x,y,z,i");

for (TableRow row : inputTable.rows()) { 
  range = row.getFloat(0);
  azimuth = row.getFloat(1);
  elevation = row.getFloat(2);
  intensity = row.getFloat(3);
  
  if (debugCount % 1000 == 0) {
    println("Debug in: " + range + ", " + azimuth + ", " + elevation + ", " + intensity);
  }
  
  //Only do conversion if range is within capability < 50m
  //If measurement is below 0.15 we get strange values
  if(range < 50.0) {
    x = range*sin(radians(elevation))*cos(radians(azimuth));
    y = range*sin(radians(elevation))*sin(radians(azimuth));
    z = range*cos(radians(elevation));
    //i = intensity;
    i = 128.0;
  
    if (debugCount % 1000 == 0) {
      println("Debug out: " + x + ", " + y + ", " + z + ", " + i);
    }
  
    //Save memory and write append to output file
    myOutput.println(x + "," + y + "," + z + "," + i);
    //output for debug
    //println(x + "," + y + "," + z + "," + i);
  }
  
  //TableRow newRow = outputTable.addRow();
  //newRow.setFloat("x", x);
  //newRow.setFloat("y", y);
  //newRow.setFloat("z", z);
  //newRow.setFloat("i", i);
  debugCount++;
}

println("Calcluated all Values");

myOutput.flush();
myOutput.close();
//saveTable(outputTable, "output/OL1xyz.CSV");

println("Saved. Done.");

exit();