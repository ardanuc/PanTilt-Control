#define DEBUGFLAG 1

// AXIS LABELS
enum AxisLabelType {AZIM_AXIS = 1, ELEV_AXIS = 2};

//function prototypes
void TurnOffSelectedMirror();

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

const int AxDeadZone_EXT1[] = {300, -780};
const int AxDeadZone_EXT2[] = {40, 780} ;
// error correction residual during the feedback
const int AxFeedbackEpsilon[] = {10, 10};



// move to a Target Location
// This function
//TargetAxisLabel is an enum of AxisLabelType Class taking in values of AZIM_AXIS and ELEV_AXIS
// TargetReading is the deisted Target Angle in degrees
// ReadFunctionPt is the pointer to the function that returns an integer value for
// MovePositiveFunctionPt allows you to move towards EXT1 within the active zone and EXT2 within the deadzone,
// likewise MoveNegativeFunctionPt allows you to move towards EXT2 within the negative zone and EXT1 within the deadzone

void movetoTarget_1D_angular ( const int TargetReading,
                               float  (*ReadFunctionPt) (void),
                               void (*MovePositiveFunctionPt) (void),
                               void (*MoveNegativeFunctionPt) (void) )
{
  static int CurrentReading;
  static short axisIndex = 0;
  // Low Pass filtered version
  static int CurrentReading_LP;
  // Low pass moving average numberof samples
  static const short Nsamples_LP = 5;
  static int data_array_LP[Nsamples_LP];
  // temporary variable to store the sum of the variables
  static long temp_LP = 0;
  //index for LP filter
  static short index_LP;

  // Realizable Target Reading based on the dead zone limits
  static int TargetReading_R;
  // The angular extremes of the deadzone
  static int ext_1, ext_2, epsilon_axis;
  ext_1 = AxDeadZone_EXT1[axisIndex]%360;
  ext_2 = AxDeadZone_EXT2[axisIndex]%360;
  epsilon_axis = AxFeedbackEpsilon[axisIndex];

  // flag to indicate dead zone
  static bool flag_dead_zone;
  //initialize to false initially
  flag_dead_zone = false;

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
  // extreme point
  
 
  if (!flag_dead_zone)
  {
    // not in dead zone, so no problem
    TargetReading_R = TargetReading;
  }

  else
  {
    TargetReading_R = (((ext_2-TargetReading)%360) > ((TargetReading-ext_1)%360))
                ? ext_1 : ext_2;
  }

  //Read the current value
  CurrentReading = ReadFunctionPt();

  //initialize the low pass filtered version with this inital reading

  for (index_LP = 0; index_LP < Nsamples_LP; index_LP++)
  {
    data_array_LP[index_LP] = CurrentReading;
  }

  //initialize the sum
  temp_LP = Nsamples_LP * CurrentReading;
 
 
 // Look at the target value and try to go to the closerl

/*
  // main feedback loop
  Serial.println("Moving: START!");

  while (abs(CurrentReading_LP - TargetReading_R) > epsilon_axis)
  {
    if (CurrentReading >= TargetReading_R)
      MoveNegativeFunctionPt();
    else
      MovePositiveFunctionPt();

    //Read the current value
    CurrentReading = ReadFunctionPt();



    // Update the Low pass version
    index_LP = (index_LP + 1) % Nsamples_LP;
    temp_LP += CurrentReading - data_array_LP[index_LP];
    data_array_LP[index_LP] = CurrentReading;

    CurrentReading_LP = (int)(temp_LP / (long)Nsamples_LP);

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


}

*/

 #if DEBUGFLAG
    Serial.print(" ext1: ");
    Serial.print(ext_1);
    Serial.print(" ext2: ");
    Serial.print(ext_2);
    Serial.print(" ACTU: ");
    Serial.print(CurrentReading);
    Serial.print(" LP: ");
    Serial.print(CurrentReading_LP);
    Serial.print(" Target: ");
    Serial.print(TargetReading);
    Serial.print(" Target-R: ");
    Serial.println(TargetReading_R);
    Serial.print(" flag_dead_zone");
    Serial.println(flag_dead_zone);
    
    #endif
    
      Serial.println("Moving: DONE!");
    
  //Since we are done-- turn off the motor
  TurnOffSelectedMirror();


}


// move to a Target Location,


// TargetReading is the desired Sensor Reading,whic can be in milig
// ReadFunctionPt is the pointer to the function that returns an integer value for
// MovePositiveFunctionPt allows you to move towards EXT1 within the active zone and EXT2 within the deadzone,
// likewise MoveNegativeFunctionPt allows you to move towards EXT2 within the negative zone and EXT1 within the deadzone

void movetoTarget_1D_linear ( const int TargetReading,
                              int  (*ReadFunctionPt) (void),
                              void (*MovePositiveFunctionPt) (void),
                              void (*MoveNegativeFunctionPt) (void) )
{
  static int CurrentReading;
  static short axisIndex = 1;
  // Low Pass filtered version
  static int CurrentReading_LP;
  // Low pass moving average numberof samples
  static const short Nsamples_LP = 5;
  static int data_array_LP[Nsamples_LP];
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
  epsilon_axis = AxFeedbackEpsilon[axisIndex];

  // flag to indicate dead zone
  static bool flag_dead_zone;
  //initialize to false initially
  flag_dead_zone = false;


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

  for (index_LP = 0; index_LP < Nsamples_LP; index_LP++)
  {
    data_array_LP[index_LP] = CurrentReading;
  }

  //initialize the sum
  temp_LP = Nsamples_LP * CurrentReading;


  // Look at the target value and try to go to the closerl

  // main feedback loop
  Serial.println("Moving!");

  while (abs(CurrentReading_LP - TargetReading_R) > epsilon_axis)
  {
    if (CurrentReading >= TargetReading_R)
      MoveNegativeFunctionPt();
    else
      MovePositiveFunctionPt();

    //Read the current value
    CurrentReading = ReadFunctionPt();



    // Update the Low pass version
    index_LP = (index_LP + 1) % Nsamples_LP;
    temp_LP += CurrentReading - data_array_LP[index_LP];
    data_array_LP[index_LP] = CurrentReading;

    CurrentReading_LP = (int)(temp_LP / (long)Nsamples_LP);

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

  }

  //Since we are done-- turn off the motor
  TurnOffSelectedMirror();

  //Serial.println("You made it here");
  //Serial.print(ReadFunctionPt());

}

