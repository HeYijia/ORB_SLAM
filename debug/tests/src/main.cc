#include <stdlib.h>
#include <stdio.h>
#include "Tests.h"
void usage() {
    printf("Usage:\n");
    printf("<bin> input-image [output-image]\n");
}

int main(int argc, char* argv[]) {
    Tests t;
    char *savefile = "ScharrOutput.jpg";
    
    switch (argc) {
    case 3:
        savefile = argv[2];
    case 2:
        t.SetTestImage(argv[1]);
        break;
    default:
        usage(); 
        return EXIT_FAILURE;
    }
    
    printf("Running tests...\n\n");

    // test scharr operator
    printf("Running Scharr test...\n");
    t.cv_scharr(savefile);

    return 0;
}
