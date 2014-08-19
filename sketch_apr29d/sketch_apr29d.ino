
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(slaveAddress);
        Wire.write("A");              // The "Get Data" command
        Wire.endTransmission();
        delay(10);                   // The HMC6352 needs at least a 70us (microsecond) delay
        // after this command.  Using 10ms just makes it safe
        // Read the 2 heading bytes, MSB first
        // The resulting 16bit word is the compass heading in 10th's of a degree
        // For example: a heading of 1345 would be 134.5 degrees
        Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
        i = 0;
        while(Wire.available() && i < 2)
        { 
          headingData[i] = Wire.read();
          i++;
        }
        headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together
        Serial.print("Current heading: ");
        Serial.print(int (headingValue / 10));     // The whole number part of the heading
        //currentHeading=headingValue/10;
        Serial.print(".");
        //Serial.print(int (headingValue % 10));     // The fractional part of the heading
        Serial.println(" degrees");
        delay(100);

/*
 if (Serial.available() > 0) //always looks for signal
        {
            incomingByte = Serial.read();
              // say what you got:
            if(incomingByte==(115))
            {
              startNumber=1;
              Serial.print("Ok!");
              lineReturn(); 
              Esc.write(100);
              //incomingByte=0;
            }
        }
 //Whenever we enter e into the Xbee program, then it will turn off this part of the loop
     if (Serial.available() > 0) //always looks for signal
      {
           // read the incoming byte:
          incomingByte = Serial.read();
           if(incomingByte==(101))
              {
                startNumber=0;
                Esc.write(0);
                //incomingByte=0;
             }     
      }               
       printTest=getHeading();
      Serial.print(printTest);
      lineReturn();
 */
