#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <hidapi.h>

#define MAX_STR 255

int main()
{

  unsigned short vend_id = 0x3553;
  unsigned short prod_id = 0xb001;

	int res;
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i;

	// Initialize the hidapi library
	res = hid_init();

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(vend_id, prod_id, NULL);
	if (!handle) {
		printf("Unable to open device\n");
		hid_exit();
 		return 1;
	}

  // Read requested state
  res = hid_read(handle, buf, 65);

  switch (buf[3])
  {
  case 0:
    printf("off\n");
    break;
  case 4:
    printf("4\n");
    break;
  case 5:
    printf("5\n");
    break;
  case 6:
    printf("6\n");
    break;

  default:
    break;
  }

	// Close the device
	hid_close(handle);

	// Finalize the hidapi library
	res = hid_exit();

  return 0;
}
