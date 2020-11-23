

#ifndef G711_H_
#define G711_H_

unsigned char linear2alaw(int);
int alaw2linear(unsigned char a_val);

unsigned char linear2ulaw(int);
int ulaw2linear(int);

#endif /*G711_H_*/
