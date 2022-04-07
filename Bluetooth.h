void bluetooth(){
  while ( insize = Serial.available() > 0) {
    buf = Serial.read();
    switch (buf)
    {
      case '"':
        RW = 1;
            // Serial.print(b++);  Serial.print("\t");
       // Serial.print("!");  Serial.print("\t");
        break;      
      case '*':
      if((cmmd[0]+cmmd[1]+cmmd[2]+cmmd[3])%100==cmmd[4]){  
       /*
          Serial.print( cmmd[0]); Serial.print("\t");
            Serial.print( cmmd[1]); Serial.print("\t");
            Serial.print( cmmd[2]); Serial.print("\t");
            Serial.print( cmmd[3]); Serial.print("\t");
              Serial.print( cmmd[4]); Serial.print("\t");
            Serial.print("\n");
     */
    cmmd[5]=cmmd[0];
    cmmd[6]=cmmd[1];
    cmmd[7]=cmmd[2];
    cmmd[8]=cmmd[3];
//ratep_yaw=cmmd[6];
//ratei_yaw=cmmd[7];
//kka=cmmd[8];
      }
        addr = 0;
        RW = 0; 
        break;
      default:
        if (RW ==1)  {
          cmmd[ addr] = buf;    
          addr++;
        }    
        }

    }

}

