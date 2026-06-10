// Tests for the AT+QENG="servingcell" response parser.
#include "sim7020e.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

static void test_lte_servingcell_israel(void) {
    // Field order per dev plan Section 4.4: MCC,MNC,TAC,CID,...
    const char *resp =
        "\r\n+QENG: \"servingcell\",\"NOCONN\",\"LTE\",\"FDD\","
        "425,01,2B4A,1234,300,5,5,-95,-11,-65,12\r\n\r\nOK\r\n";
    CellFix_t c;
    CHECK(sim7020e_parse_servingcell(resp, &c));
    CHECK(c.valid);
    CHECK(c.mcc == 425);
    CHECK(c.mnc == 1);
    CHECK(c.lac == 0x2B4A);
    CHECK(c.cid == 0x1234);
}

static void test_lebanon_mcc(void) {
    const char *resp =
        "+QENG: \"servingcell\",\"LTE\",\"FDD\",415,03,1A2B,0,5678,300,5\r\nOK";
    CellFix_t c;
    CHECK(sim7020e_parse_servingcell(resp, &c));
    CHECK(c.mcc == 415); // Lebanon — would trigger spoofing detection
    CHECK(c.mnc == 3);
}

static void test_no_qeng_marker(void) {
    CellFix_t c;
    CHECK(!sim7020e_parse_servingcell("ERROR\r\n", &c));
}

static void test_null_inputs(void) {
    CellFix_t c;
    CHECK(!sim7020e_parse_servingcell(NULL, &c));
    CHECK(!sim7020e_parse_servingcell("+QENG: x", NULL));
}

int main(void) {
    printf("test_at_parser\n");
    test_lte_servingcell_israel();
    test_lebanon_mcc();
    test_no_qeng_marker();
    test_null_inputs();
    if (g_failures == 0) {
        printf("  all passed\n");
        return 0;
    }
    printf("  %d failure(s)\n", g_failures);
    return 1;
}
