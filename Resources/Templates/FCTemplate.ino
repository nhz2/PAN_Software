//
// Resources/Templates/FCTemplate.ino
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

/*!
 */

/*!
 */
#define FCT_VERBOSE

/*!
 */
#define FCT_TESTING

// Macro Congifuration ---------------------------------------------------------

#ifdef FCT_TESTING
#ifndef FCT_VERBOSE
#define FCT_VERBOSE
#endif // FCT_VERBOSE
/*!
 */
void fct_test_on_char(unsigned char code);

/*!
 */
void fct_empty_serial() {
  while(Serial.available())
    Serial.read();
}
#endif // FCT_TESTING

// Flight Code Initiation ------------------------------------------------------

void setup() {

  /*!
   */

  #ifdef FCT_VERBOSE
  /*!
   */
  Serial.begin(9600);
  #ifdef FCT_TESTING
  /*!
   */
  while(1) {
    Serial.println("$Ready for test charactar input");
    while(!Serial.available());
    unsigned char code = Serial.read();
    fct_empty_serial();
    fct_test_on_char(code);
  }
  #endif
  #endif

}

void loop() {
  /*!
   */
}

#ifdef FCT_TESTING

// Test Cases ------------------------------------------------------------------

/*!
 */

inline namespace fct_v2 {

  void test_a(int &x) {
    x += 5;
  }

}

void fct_test_on_char(unsigned char code) {
  switch (code) {
    case 'a':
      test_a();
      break;
  }
}

// Outdated test cases ---------------------------------------------------------

namespace fct_v1 {

  void test_a(int &x) {
    x = 5;
  }

}

#endif
