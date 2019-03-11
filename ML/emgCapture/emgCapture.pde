import processing.serial.*;

Serial mySerial; 
Table table; 
String val; 

int windowCount = 1; 
// int sampleCount = 1; 
int Time = 0; 

boolean written = false; // used for the table

boolean firstContact = false;  // indicates whether or not hand shake connection has been established

//boolean state = false;         // is recording or not recording 
boolean declared = false;      // state has been declared

void setup() { 
  String portName = Serial.list()[0]; 
  println("Port Name:", portName); 
  mySerial = new Serial(this, portName, 115200); 
  mySerial.bufferUntil('\n'); 
  table = new Table(); 
  
  table.addColumn("Note", Table.STRING); 
  // table.addColumn("Sample", Table.STRING); 
  table.addColumn("Value", Table.STRING);
   // table.addColumn("Value", Table.FLOAT);
  
  size(400, 400); 
}

// executes continuously (like loop in Arduino)
void draw() {
  
  if (keyPressed) {
    delay(200); 
    if (key == 'e') {
      
      mySerial.write('e'); 
      TableRow endRow = table.addRow(); 
      
      endRow.setString("Note", "End of recording - " + (hour() + ":" + minute() + ":" + second()));
      // endRow.setString("Sample", );
      saveTable(table, "Mai - clench.csv"); 
      exit(); 
    }
  }
  
  if (firstContact) {
      if (mySerial.available() > 0) { 
        if (!declared) {
          println("Recording");
          TableRow newRow = table.addRow(); 
          
          newRow.setString("Note", ("Window " + windowCount)); 
          // newRow.setString("Sample", "Beginning sample"); 
          windowCount++; 
          declared = true; 
        }
        
        // println("Getting EMG data"); 
        String emgIn = serialExtract();
      }   
  } else {
    if (mySerial.available() > 0) {
      String sValue = serialExtract();
      complete(mySerial, sValue); 
      if (firstContact) {
        println("Ready");
      } else {
        println("Waiting");
      }
    }
  }  
}

// return serial data
String serialExtract() {
  String val = mySerial.readStringUntil('\n'); 
  
  if (val != null) { 
    // Trim gets rid of any unnecessary spaces around the main string
    val = trim(val);
    
    if (!firstContact) {
      println("Bro"); 
    } else { 
        if (val.charAt(0) == 'v') {
          
          TableRow addData = table.addRow();
          String[] data = split(val, ','); 
        
          // println(data[0]);
          // println(data[1]); 
          
          // addData.setString("Sample", str(sampleCount));
          // sampleCount++; 
          addData.setString("Value", data[1]);        
      } else if(val.charAt(0) == 's') {
        
        TableRow addData = table.addRow();
        TableRow endSection = table.addRow();
        
        String[] data = split(val, ','); 
      
        // println(data[0]);
        // println(data[1]); 
        
        // addData.setString("Sample", str(sampleCount));
        addData.setString("Value", data[1]); 
        
        // endSection.setString("Sample", "End of window"); 
        endSection.setString("Note", "End of window"); 
        endSection.setString("Note", "Window " + windowCount);
        windowCount++; 
        // sampleCount = 1;
      }
      
      val = "";
    }
  }
  
  // println("Returning serialExtract()"); 
  return val;
}

// complete Handshake
void complete(Serial mySerial, String in) {
  if (in != null) {
    
    if (in.equals("b")) {
      for (int i = 0; i < 5; i++) {
        mySerial.write('f');
        delay(10); 
      } 
      
      firstContact = true; 
      println("Contact established");
      clearBuffer(mySerial); 
      TableRow addData = table.addRow();
      
      addData.setString("Note", ("Begin Recording - " + (hour() + ":" + minute() + ":" + second())));
      // addData.setString("Sample", ); 
      println("Begin recording"); 
      // delay(50);      
    }          
  }
}

void clearBuffer(Serial mySerial) {
  boolean bCheck = false; 
  while (!bCheck) {
    if (mySerial.available() > 0) {
      if (mySerial.readStringUntil('\n') == "b") {
        String _ = mySerial.readStringUntil('\n'); 
      } else {
        bCheck = true; 
      }      
    }
    mySerial.clear();   
  }
}
