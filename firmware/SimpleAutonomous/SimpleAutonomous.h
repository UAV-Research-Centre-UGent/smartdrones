// Debugging switches and macros
//#if DEBUG
#define PRINTS(s)     { Serial.println(F(s)); }
#define PRINTEOL()    { Serial.println(""); }
#define PRINT(s,v)    { Serial.print(F(s)); Serial.print(v); }
#define PRINTX(s,v)   { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); }
#define PRINTCMD(s,c) { Serial.print(F(s)); \
                        Serial.print(F("[")); \
                        Serial.print((char)c.cmd); \
                        Serial.print(F(",")); \
                        Serial.print(isalnum(c.data) ? (char)c.data : c.data); \
                        Serial.print(F("] ")); \
                      }
//#else
//#define PRINTS(s)
//#define PRINT(s,v)
//#define PRINTX(s,v)
//#define PRINTCMD(s,c)
//#define PRINTFSM(s,f)
//#endif
