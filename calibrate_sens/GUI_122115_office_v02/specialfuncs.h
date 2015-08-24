
          //function prototypes
          void TurnOffSelectedMirror();
          void MoveRightSelectedMirror (void);
          void MoveLeftSelectedMirror (void) ;
          void MoveUpSelectedMirror (void) ;
          void MoveDownSelectedMirror (void) ;
          int ReadAZIM_PotentiometerEncoder();
          int ReadELEV_PotentiometerEncoder();
          void ReadSerialCommand(void);
          int ExecuteMildCommandSet(void);
          void UpdateMotorActiveBooleans(void);
          int ReadandExecuteMildCommandSet(void);
          void ResetInputString(void); 


          /* Minimum values of AXIS
          First axis maximum/minimum is azimuthal limitsof the dead zone
          Second axis maximum/minimum is tilt lmits in milig
          For angular deadzones with a periodicity of 360, detemination of whether you
          are in the deadzone or not is explained is explained in Suntomics
          Research notebook
          dated 06/27/2014, page 20

          For angular variables, the range of motion is divided into deadzone and activezone which
          are complementary and whose sum spans the total 360 degrees.

          EXT1 and EXT2 are not random
          I will assume that
          AxDeadZone_EXT1 (shorthand EXT1) :
          The angular value between 0 and 360 which is the
          which is the first boundary you hit if you rotate clockwise, when you are in the active region

          AxDeadZone_EXT2 (shorthand EXT2) :
          The angular value between 0 and 360 which is the
          which is the first boundary you hit if you rotate ANTIclockwise, when you are in the active region

          Note that within the active region EXT2 will always be in your counterclockwise (negative sense)
          and EXT1 will be in your clockwise (positive) direction

          ---
          For linear variables such as acceleration,
          assume EXT1 is the minimum vaue of te sensor reading and EXT2 is the maximum

          */

          // All angles are in units of 0.01 degrees
          /* For AxDeadZone_EXT(1,2) and AxFeedbackEpsilon
          each column represents a different sensor asociated with axis

          Column-1 : AZIMUTH axis using magnetic heading function
                     --angular, 36000 (360 degrees) periodicity, long
          Column-2 : ELEVATION axis using tiltsensor Y-axis Acceleration function
                     --linear
          Column-3 : AZIMUTH axis using potentiometer ENCODER Reading
                     --linear
          Column-4 : ELEVATION axis using potentiometer ENCODER Reading
                     --linear
          */

    // set extreme points to very non limiting values
                     long AxDeadZone_EXT1[] = {400L, -780, 500, 501};
                     long AxDeadZone_EXT2[] = {2900L, 780, 502, 503} ;


    /*
          #if PIXEL_ID==1
                     int AxDeadZone_EXT1[] = {400, -780, 0, 369};
                     int AxDeadZone_EXT2[] = {2900, 780, 984, 668} ;
          #endif


          #if PIXEL_ID==2
                     int AxDeadZone_EXT1[] = {400, -780, 17, 350};
                     int AxDeadZone_EXT2[] = {2900, 780, 1023, 664} ;
          #endif


          #if PIXEL_ID==3
                     int AxDeadZone_EXT1[] = {400, -780, 0, 374};
                     int AxDeadZone_EXT2[] = {2900, 780, 985, 675} ;
          #endif


          #if PIXEL_ID==4
                     int AxDeadZone_EXT1[] = {400, -780, 0, 498};
                     int AxDeadZone_EXT2[] = {2900, 780, 1009, 784} ;
          #endif

          #if PIXEL_ID==5
                     int AxDeadZone_EXT1[] = {400, -780, 0, 353};
                     int AxDeadZone_EXT2[] = {2900, 780, 1008, 667} ;
          #endif

          #if PIXEL_ID==6
                     int AxDeadZone_EXT1[] = {400, -780, 19, 383};
                     int AxDeadZone_EXT2[] = {2900, 780, 1016, 691} ;
          #endif

          #if PIXEL_ID==7
                     int AxDeadZone_EXT1[] = {400, -780, 0, 337};
                     int AxDeadZone_EXT2[] = {2900, 780, 992, 643} ;
          #endif

          #if PIXEL_ID==8
                     int AxDeadZone_EXT1[] = {400, -780, 0, 391};
                     int AxDeadZone_EXT2[] = {2900, 780, 993, 671} ;
          #endif

          #if PIXEL_ID==9
                     int AxDeadZone_EXT1[] = {400, -780, 0, 362};
                     int AxDeadZone_EXT2[] = {2900, 780, 999, 668} ;
          #endif

          #if PIXEL_ID==-1
                     int AxDeadZone_EXT1[] = {400, -780, 0, 362};
                     int AxDeadZone_EXT2[] = {2900, 780, 999, 668} ;
          #endif

    */


          

          /*
          // error correction residual during the feedback
          // Units are

          Column-1 : AZIMUTH axis using magnetic heading function
                     --angular, Units 0.01 degrees
          Column-2 : ELEVATION axis using tiltsensor Y-axis Acceleration function
                     --linear -UNIT: 1 mg
          Column-3 : AZIMUTH axis using potentiometer ENCODER Reading
                     --linear -UNIT: Least Significant Bi
          Column-4 : ELEVATION axis using potentiometer ENCODER Reading
                     --linear -UNIT: Least Significant Bit
          */
                     const int AxFeedbackEpsilon[] = {1500, 10, 0, 0};

          //keep converting from float to degrees

          // move to a Target Location
          // This function
          //TargetAxisLabel is an enum of AxisLabelType Class taking in values of AZIM_AXIS and ELEV_AXIS
          // TargetReading is the deisted Target Angle in degrees
          // ReadFunctionPt is the pointer to the function that returns an integer value for
          // MovePositiveFunctionPt allows you to move towards EXT1 within the active zone and EXT2 within the deadzone,
          // likewise MoveNegativeFunctionPt allows you to move towards EXT2 within the negative zone and EXT1 within the deadzone

                     void movetoTarget_1D_angular ( const short & axisIndex, const int TargetReading,
                       long  (*ReadFunctionPt) (void),
                       void (*MovePositiveFunctionPt) (void),
                       void (*MoveNegativeFunctionPt) (void) )
                     {
                      static long CurrentReading;

            // Low Pass filtered version
                      static long CurrentReading_LP;

                      static long data_array_LP[NSAMPLES_LP];
            // temporary variable to store the sum of the variables
                      static long temp_LP = 0;
            //index for LP filter
                      static short index_LP;

            // Realizable Target Reading based on the dead zone limits
                      static long TargetReading_R;
            // The angular extremes of the deadzone
                      static long ext_1, ext_2, epsilon_axis;
                      ext_1 = AxDeadZone_EXT1[axisIndex] %36000L;
                      ext_2 = AxDeadZone_EXT2[axisIndex] % 36000L;
                      epsilon_axis = AxFeedbackEpsilon[axisIndex];

            // flag to indicate dead zone
                      static bool flag_dead_zone;
            //initialize to false initially
                      flag_dead_zone = false;


              // Clear the Serial Read String since other Serial commands might interrupt
             // during the feedback loop;
            ResetInputString(); 
           

            //Read the current value
                      CurrentReading = ReadFunctionPt();
            // Look at the target value and try to go to the closerl

            // First determine if TargetReading  is in the deadzone or not
            // analyze in two cases based on the modulus 360 change
                      if (ext_2 > ext_1)
            { // This is the regular case
              flag_dead_zone = (TargetReading > ext_1) && (TargetReading < ext_2);
            }
            else
            { // This is the case when there is a modulus change between ext_2 and ext_@
              flag_dead_zone = !((TargetReading > ext_2) && (TargetReading < ext_1));
            }

            // If the target is in the dead zone, it is also desired to move tothe closest
            // extreme point. Since I am using integermodula operator, accuracy is to within a degree


            if (!flag_dead_zone)
            {
              // not in dead zone, so no problem
              TargetReading_R = TargetReading % 36000;
            }

            else
            {
              TargetReading_R = (((int)(ext_2 - TargetReading) % 36000L) > ((int)(TargetReading - ext_1) % 36000L))
              ? ext_1 : ext_2;
            }

            //Read the current value
            CurrentReading = ReadFunctionPt() % 36000;

            //initialize the low pass filtered version with this inital reading

            for (index_LP = 0; index_LP < NSAMPLES_LP; index_LP++)
            {
              data_array_LP[index_LP] = CurrentReading;
            }

            //initialize the sum
            temp_LP = NSAMPLES_LP * CurrentReading;

            // update the low passed version
            CurrentReading_LP = CurrentReading;

            // Look at the target value and try to go to the closerl


            // main feedback loop
          #if DEBUGFLAG
            Serial.println("Moving: START!");

          #endif

            while (abs(CurrentReading - TargetReading_R) > epsilon_axis)
            {
              if
                (
                  ((CurrentReading <= TargetReading_R) &&
                   ((ext_1 >= TargetReading_R) || (ext_1 <= CurrentReading)))
                  ||
                  ((CurrentReading >= ext_1) && (ext_1 >= TargetReading))
                  ||
                  ((CurrentReading >= TargetReading_R) && (ext_1 >= TargetReading_R))
                  )

              {
          #if DEBUGFLAG
                Serial.print("Running positive function");
          #endif
                MovePositiveFunctionPt();
              }
              else
              {
          #if DEBUGFLAG
                Serial.print("Running negative function");
          #endif
                MoveNegativeFunctionPt();
              }

              //Read the current value
              CurrentReading = ReadFunctionPt() % 36000;

          #if PERFORM_LPF_PROCESSING ==1

              // Update the Low pass version
              index_LP = (index_LP + 1) % NSAMPLES_LP;
              temp_LP += CurrentReading - data_array_LP[index_LP];
              data_array_LP[index_LP] = CurrentReading;

              CurrentReading_LP = (temp_LP / (long)NSAMPLES_LP);

          #endif

          #if DEBUGFLAG
              Serial.print(" ACTU: ");
              Serial.print(CurrentReading);
              Serial.print(" LP: ");
              Serial.print(CurrentReading_LP);
              Serial.print(" Target: ");
              Serial.print(TargetReading);
              Serial.print(" Target-R: ");
              Serial.print(TargetReading_R);
              Serial.print(" ext-1: ");
              Serial.print(ext_1);
              Serial.print(" ext-2: ");
              Serial.println(ext_2);


          #endif

              // See if there are other commands sent to serial port
              // read the command if there s any Serial bbyte available
              // Note that any STOP command (STOP, STOPUD, STOPLR, STOPALL)  will act like STOPALL
              // due to TurnOffSelectedMirror(); at the end

              if (Serial.available() > 0) {
                // Exit the while feedback loop, if there was an emergency stop command
                   
                  

                if (1==ReadandExecuteMildCommandSet())
                {
                  #if DEBUGFLAG
                  Serial.println("STOP RECEIVED: EXITING LOOP!");
                  #endif
                 break;
                }

              } //If Serial.available()


            } 



          #if DEBUGFLAG


            Serial.println("Moving: DONE!");

          #endif

            //Since we are done-- turn off the motor
            TurnOffSelectedMirror();


          }


          // move to a Target Location,


          // TargetReading is the desired Sensor Reading,whic can be in milig
          // ReadFunctionPt is the pointer to the function that returns an integer value for
          // MovePositiveFunctionPt allows you to move towards EXT1 within the active zone and EXT2 within the deadzone,
          // likewise MoveNegativeFunctionPt allows you to move towards EXT2 within the negative zone and EXT1 within the deadzone

          void movetoTarget_1D_linear ( const short & axisIndex, const int TargetReading,
            int  (*ReadFunctionPt) (void),
            void (*MovePositiveFunctionPt) (void),
            void (*MoveNegativeFunctionPt) (void) )
          {
            static int CurrentReading;


            // Low Pass filtered version
            static int CurrentReading_LP;

            static int data_array_LP[NSAMPLES_LP];
            // temporary variable to store the sum of the variables
            static long temp_LP = 0;
            //index for LP filter
            static short index_LP;

       
           


            // Realizable Target Reading based on the dead zone limits
            static int TargetReading_R;
            // The angular extremes of the deadzone
            static int ext_1, ext_2, epsilon_axis;
            ext_1 = AxDeadZone_EXT1[axisIndex];
            ext_2 = AxDeadZone_EXT2[axisIndex];
            epsilon_axis = (int) AxFeedbackEpsilon[axisIndex];

            // flag to indicate dead zone
            static bool flag_dead_zone;
            //initialize to false initially
            flag_dead_zone = false;

              // Clear the Serial Read String since other Serial commands might interrupt
             // during the feedback loop;
            ResetInputString(); 


            // First determine if TargetReading  is in the deadzone or not
            // analyze in two cases based on the modulus 360 change
            if (!(ext_2 >= ext_1))
            { // in case ext_2 is not entered as the maximum , swap the variables
              ext_2 = AxDeadZone_EXT1[axisIndex];
              ext_1 = AxDeadZone_EXT2[axisIndex];

            }


            flag_dead_zone = (TargetReading < ext_1) || (TargetReading > ext_2);

            // If the target is in the dead zone, it is also desired to move tothe closest
            if (!flag_dead_zone)
            {
              // not in dead zone, so no problem
              TargetReading_R = TargetReading;
            }

            else
            {
              TargetReading_R = (TargetReading < ext_1) ? ext_1 : ext_2;
            }

            //Read the current value
            CurrentReading = ReadFunctionPt();



            //initialize the low pass filtered version with this inital reading

            for (index_LP = 0; index_LP < NSAMPLES_LP; index_LP++)
            {
              data_array_LP[index_LP] = CurrentReading;
            }

            //initialize the sum
            temp_LP = NSAMPLES_LP * CurrentReading;

            // update the low passed version
            CurrentReading_LP = CurrentReading;


            // Look at the target value and try to go to the closerl

            // main feedback loop
          #if DEBUGFLAG
            Serial.println("Moving!");

          #endif

            while (abs(CurrentReading - TargetReading_R) > epsilon_axis)
            {
              if (CurrentReading >= TargetReading_R)
              {
          #if DEBUGFLAG
                Serial.print("Running negative function");
          #endif
                MoveNegativeFunctionPt();
              }
              else
              {
          #if DEBUGFLAG
                Serial.print("Running positive function");
          #endif
                MovePositiveFunctionPt();
              }
              //Read the current value
              CurrentReading = ReadFunctionPt();


          #if PERFORM_LPF_PROCESSING ==1

              // Update the Low pass version
              index_LP = (index_LP + 1) % NSAMPLES_LP;
              temp_LP += CurrentReading - data_array_LP[index_LP];
              data_array_LP[index_LP] = CurrentReading;

              CurrentReading_LP = (int)(temp_LP / (long)NSAMPLES_LP);

          #endif

          #if DEBUGFLAG
              Serial.print(" ACTU: ");
              Serial.print(CurrentReading);
              Serial.print(" LP: ");
              Serial.print(CurrentReading_LP);
              Serial.print(" Target: ");
              Serial.print(TargetReading);
              Serial.print(" Target-R: ");
              Serial.println(TargetReading_R);

          #endif
              
              // See if there are other commands sent to serial port
              // read the command if there s any Serial bbyte available
              // Note that any STOP command (STOP, STOPUD, STOPLR, STOPALL)  will act like STOPALL
              // due to TurnOffSelectedMirror(); at the end

              if (Serial.available() > 0) {
                // Exit the while feedback loop, if there was an emergency stop command
                   

                if (1==ReadandExecuteMildCommandSet())
                {
                  #if DEBUGFLAG
                  Serial.println("STOP RECEIVED: EXITING LOOP!");
                  #endif
                 break;
                }

              } //If Serial.available()


            }

          #if DEBUGFLAG
            Serial.println("Moving: DONE!");
          #endif

            //Since we are done-- turn off the motor
            TurnOffSelectedMirror();

            //Serial.println("You made it here");
            //Serial.print(ReadFunctionPt());

          }


          // movetoTarget_2D_linear runs azimuth and levation feedback subroutines
          // consecutively based on target potentiometer readings.
          // It runs both feedbacks simultaneously to make it quicker than consecutiv running



          void movetoTarget_2D_linear ( const int TargetReading_AZIM, const int TargetReading_ELEV)
          // @param: TargetReading_AZIM:  Targeted Potentiometer Reading of the AZIMUTh Axis
          // @param: TargetReading_ELEV:  Targeted Potentiometer Reading
          {



            /*
                    // Positive and Negative Function Pointers for Azim and ELEV

                   // Pointer to function to action towards increasing the Value Read by ReadFunctionPt()
                  MovePositiveFunctionPt_AZIM = &MoveRightSelectedMirror;

                  // Pointer to function to action towards decreasing the Value Read by ReadFunctionPt()
                  MoveNegativeFunctionPt_AZIM= &MoveLeftSelectedMirror;

                  // Pointer to function to action towards increasing the Value Read by ReadFunctionPt()
                  MovePositiveFunctionPt_ELEV = &MoveUpSelectedMirror;

                  // Pointer to function to action towards decreasing the Value Read by ReadFunctionPt()
                  MoveNegativeFunctionPt_ELEV= &MoveDownSelectedMirror;

            */
                  static void (*MovePositiveFunctionPt_AZIM) (void) = &MoveRightSelectedMirror;
                  static void (*MoveNegativeFunctionPt_AZIM) (void) = &MoveLeftSelectedMirror;
                  static void (*MovePositiveFunctionPt_ELEV) (void) = &MoveUpSelectedMirror;
                  static void (*MoveNegativeFunctionPt_ELEV) (void) = &MoveDownSelectedMirror;

                  static int CurrentReading_AZIM, CurrentReading_ELEV;

            // Low Pass filtered version
                  static int CurrentReading_LP_AZIM, CurrentReading_LP_ELEV;

                  static int data_array_LP_AZIM[NSAMPLES_LP], data_array_LP_ELEV[NSAMPLES_LP];
            // temporary variable to store the sum of the variables
                  static long temp_LP_AZIM = 0, temp_LP_ELEV = 0;
            //index for LP filter
                  static short index_LP_AZIM, index_LP_ELEV ;

            // Realizable Target Reading based on the dead zone limits
                  static int TargetReading_R_AZIM, TargetReading_R_ELEV;
            // The angular extremes of the deadzone
                  static int ext_1_AZIM, ext_2_AZIM, epsilon_axis_AZIM;
                  static int ext_1_ELEV, ext_2_ELEV, epsilon_axis_ELEV;


            // Clear the Serial Read String since other Serial commands might interrupt
             // during the feedback loop;
            ResetInputString();      

                  ext_1_AZIM = AxDeadZone_EXT1[2];
                  ext_2_AZIM = AxDeadZone_EXT2[2];
                  epsilon_axis_AZIM = (int) AxFeedbackEpsilon[2];

                  ext_1_ELEV = AxDeadZone_EXT1[3];
                  ext_2_ELEV = AxDeadZone_EXT2[3];
                  epsilon_axis_ELEV = (int) AxFeedbackEpsilon[3];

            // flag to indicate dead zone
                  static bool flag_dead_zone_AZIM = false, flag_dead_zone_ELEV = false;
            //initialize to false initially
                  flag_dead_zone_AZIM = false;
                  flag_dead_zone_ELEV = false;


            // First determine if TargetReading  is in the deadzone or not

                  if (!(ext_2_AZIM >= ext_1_AZIM))
            { // in case ext_2 is not entered as the maximum , swap the variables
              ext_2_AZIM  = AxDeadZone_EXT1[2];
              ext_1_AZIM  = AxDeadZone_EXT2[2];

            }

            if (!(ext_2_ELEV >= ext_1_ELEV))
            { // in case ext_2 is not entered as the maximum , swap the variables
              ext_2_ELEV  = AxDeadZone_EXT1[3];
              ext_1_ELEV  = AxDeadZone_EXT2[3];

            }


            flag_dead_zone_AZIM = (TargetReading_AZIM < ext_1_AZIM) || (TargetReading_AZIM > ext_2_AZIM);
            flag_dead_zone_ELEV = (TargetReading_ELEV < ext_1_ELEV) || (TargetReading_ELEV > ext_2_ELEV);

            // If the target is in the dead zone, it is also desired to move tothe closest
            if (!flag_dead_zone_AZIM)
            {
              // not in dead zone, so no problem
              TargetReading_R_AZIM = TargetReading_AZIM;
            }

            else
            {
              TargetReading_R_AZIM = (TargetReading_AZIM < ext_1_AZIM) ? ext_1_AZIM : ext_2_AZIM;
            }

            // If the target is in the dead zone, it is also desired to move tothe closest
            if (!flag_dead_zone_ELEV)
            {
              // not in dead zone, so no problem
              TargetReading_R_ELEV = TargetReading_ELEV;
            }

            else
            {
              TargetReading_R_ELEV = (TargetReading_ELEV < ext_1_ELEV) ? ext_1_ELEV : ext_2_ELEV;
            }

            //Read the current value
            CurrentReading_AZIM = ReadAZIM_PotentiometerEncoder();
            CurrentReading_ELEV = ReadELEV_PotentiometerEncoder();



            //initialize the low pass filtered version with this inital reading

            for (index_LP_AZIM = 0; index_LP_AZIM < NSAMPLES_LP; index_LP_AZIM++)
            {
              data_array_LP_AZIM[index_LP_AZIM] = CurrentReading_AZIM;
            }
            //initialize the low pass filtered version with this inital reading

            for (index_LP_ELEV = 0; index_LP_ELEV < NSAMPLES_LP; index_LP_ELEV++)
            {
              data_array_LP_ELEV[index_LP_ELEV] = CurrentReading_ELEV;
            }


            //initialize the sum
            temp_LP_AZIM = NSAMPLES_LP * CurrentReading_AZIM;
            temp_LP_ELEV = NSAMPLES_LP * CurrentReading_ELEV;

            // update the low passed version
            CurrentReading_LP_AZIM = CurrentReading_AZIM;
            CurrentReading_LP_AZIM = CurrentReading_ELEV;

            // Look at the target value and try to go to the closerl
            // Run nested  AZIM and ELEV feedback loops
            // main feedback loop
          #if DEBUGFLAG
            Serial.println("Moving!");

          #endif


            while ((abs (CurrentReading_AZIM - TargetReading_R_AZIM) > epsilon_axis_AZIM)
             || (abs(CurrentReading_ELEV - TargetReading_R_ELEV) > epsilon_axis_ELEV))

            {

              

              if (CurrentReading_AZIM >= TargetReading_R_AZIM)
              {
          #if DEBUGFLAG
                Serial.print("AZIM:Running negative function");
          #endif
                MoveNegativeFunctionPt_AZIM();
              }
              else
              {
          #if DEBUGFLAG
                Serial.print("AZIM:Running positive function");
          #endif
                MovePositiveFunctionPt_AZIM();
              }


              if (CurrentReading_ELEV >= TargetReading_R_ELEV)
              {
          #if DEBUGFLAG
                Serial.println("ELEV:Running negative function");
          #endif
                MoveNegativeFunctionPt_ELEV();
              }
              else
              {
          #if DEBUGFLAG
                Serial.println("ELEV:Running positive function");
          #endif
                MovePositiveFunctionPt_ELEV();
              }

              //Read the current value
              CurrentReading_AZIM = ReadAZIM_PotentiometerEncoder();
              CurrentReading_ELEV = ReadELEV_PotentiometerEncoder();


          #if PERFORM_LPF_PROCESSING ==1

              // Update the Low pass version
              index_LP_AZIM = (index_LP_AZIM + 1) % NSAMPLES_LP;
              temp_LP_AZIM += CurrentReading_AZIM - data_array_LP_AZIM[index_LP_AZIM];
              data_array_LP_AZIM[index_LP_AZIM] = CurrentReading_AZIM;

              CurrentReading_LP_AZIM = (int)(temp_LP_AZIM / (long)NSAMPLES_LP);

              // Update the Low pass version
              index_LP_ELEV = (index_LP_ELEV + 1) % NSAMPLES_LP;
              temp_LP_ELEV += CurrentReading_ELEV - data_array_LP_ELEV[index_LP_ELEV];
              data_array_LP_ELEV[index_LP_ELEV] = CurrentReading_ELEV;

              CurrentReading_LP_ELEV = (int)(temp_LP_ELEV / (long)NSAMPLES_LP);

          #endif

          #if DEBUGFLAG
              Serial.print(" AZIM-ACTU: ");
              Serial.print(CurrentReading_AZIM);
              Serial.print(" LP: ");
              Serial.print(CurrentReading_LP_AZIM);
              Serial.print(" Target: ");
              Serial.print(TargetReading_AZIM);
              Serial.print(" Target-R: ");
              Serial.println(TargetReading_R_AZIM);

              Serial.print(" ELEV-ACTU: ");
              Serial.print(CurrentReading_ELEV);
              Serial.print(" LP: ");
              Serial.print(CurrentReading_LP_ELEV);
              Serial.print(" Target: ");
              Serial.print(TargetReading_ELEV);
              Serial.print(" Target-R: ");
              Serial.println(TargetReading_R_ELEV);
          #endif


              // See if there are other commands sent to serial port
              // read the command if there s any Serial bbyte available
              // Note that any STOP command (STOP, STOPUD, STOPLR, STOPALL)  will act like STOPALL
              // due to TurnOffSelectedMirror(); at the end

              if (Serial.available() > 0) {
                // Exit the while feedback loop, if there was an emergency stop command
                   

                if (1==ReadandExecuteMildCommandSet())
                {
                  #if DEBUGFLAG
                  Serial.println("STOP RECEIVED: EXITING LOOP!");
                  #endif
                 break;
                }

              } //If Serial.available()

            }

          #if DEBUGFLAG
            Serial.println("Moving: DONE!");
          #endif

            //Since we are done-- turn off the motor
            TurnOffSelectedMirror();


          }
        


    //----------------
     
  void setDeadZoneEXTVal ( const bool minFlag, const short  axisIndex, const int targetValue)
    {
    // Sets the AxDeadZone_EXT1 [axisIndex]= targetValue if minFlag is true
    // Sets the AxDeadZone_EXT2 [axisIndex]= targetValue if minFlag is  false
    // @param: minFlag    : min max selesctor flag, if true changes the EXT1 value 
    // otherwise EXT2 value
    // @param: axisIndex:  Targeted Axis indicator from 0-3
    //
    //         /*  Feedback axes are determined based on the following
    // axisIndex=0, Column-1 : AZIMUTH axis using magnetic heading function
    //            --angular, 36000 (360 degrees) periodicity, long
    // axisIndex=1,Column-2 : ELEVATION axis using tiltsensor Y-axis Acceleration function
    //            --linear
    // axisIndex=2, Column-3 : AZIMUTH axis using potentiometer ENCODER Reading
    //            --linear
    // axisIndex=3, Column-4 : ELEVATION axis using potentiometer ENCODER Reading
    //            --linear
    // @param: targetValue:  Targeted min/max value
    
    if ((0<=axisIndex) && (axisIndex<=3))
        {
        if (true==minFlag)
          AxDeadZone_EXT1 [axisIndex]= targetValue;
        else
          AxDeadZone_EXT2 [axisIndex]= targetValue;
      }

    }


    // --------------------------
    int getDeadZoneEXTVal ( const bool minFlag, const short  axisIndex)
    {
    // Gets the AxDeadZone_EXT1 [axisIndex]= targetValue if minFlag is true
    // Gets the AxDeadZone_EXT2 [axisIndex]= targetValue if minFlag is  false
    // @param: minFlag    : min max selesctor flag, if true changes the EXT1 value 
    // otherwise EXT2 value
    // @param: axisIndex:  Targeted Axis indicator from 0-3
    //
    //         /*  Feedback axes are determined based on the following
    // axisIndex=0, Column-1 : AZIMUTH axis using magnetic heading function
    //            --angular, 36000 (360 degrees) periodicity, long
    // axisIndex=1,Column-2 : ELEVATION axis using tiltsensor Y-axis Acceleration function
    //            --linear
    // axisIndex=2, Column-3 : AZIMUTH axis using potentiometer ENCODER Reading
    //            --linear
    // axisIndex=3, Column-4 : ELEVATION axis using potentiometer ENCODER Reading
    //            --linear
    // @param: extremeValue:  readValue
    
    if ((0<=axisIndex) && (axisIndex<=3))
        {
        if (true==minFlag)
          return AxDeadZone_EXT1 [axisIndex];
        else
          return AxDeadZone_EXT2 [axisIndex];
      }
      else
        // axisIndex ot of bounds
        return -1;

    }
