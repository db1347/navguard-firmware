// Pure AT-response parser for the serving-cell query. No hardware deps so it
// is unit-tested on desktop. Handles the Quectel/SIMCom +QENG response forms:
//   +QENG: "servingcell","NOCONN","LTE","FDD",425,01,2B4A,...,1234,...
//   +QENG: "servingcell","LTE","FDD",425,01,2B4A,1234,...
#include "sim7020e.h"

#include <stdlib.h>
#include <string.h>

// Find the substring after the +QENG: marker, then walk comma fields. The
// first all-digit field after the band/duplex tokens is MCC, followed by MNC,
// LAC (hex), and CID (hex).
bool sim7020e_parse_servingcell(const char *resp, CellFix_t *out) {
    if (resp == NULL || out == NULL) return false;
    const char *q = strstr(resp, "+QENG:");
    if (q == NULL) return false;

    memset(out, 0, sizeof(*out));

    // Tokenize on commas into a local copy.
    char buf[256];
    strncpy(buf, q, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // Collect numeric/quoted fields in order.
    char *fields[32];
    int nf = 0;
    char *save = NULL;
    for (char *tok = strtok_r(buf, ",", &save);
         tok != NULL && nf < 32;
         tok = strtok_r(NULL, ",", &save)) {
        // strip surrounding quotes/whitespace
        while (*tok == ' ' || *tok == '"') tok++;
        size_t len = strlen(tok);
        while (len > 0 && (tok[len - 1] == '"' || tok[len - 1] == ' ' ||
                           tok[len - 1] == '\r' || tok[len - 1] == '\n')) {
            tok[--len] = '\0';
        }
        fields[nf++] = tok;
    }

    // Locate MCC: first purely-decimal field equal to a 3-digit country code,
    // immediately followed by another decimal (MNC). Scan from the start.
    for (int i = 0; i + 3 < nf; i++) {
        const char *f = fields[i];
        size_t len = strlen(f);
        if (len < 3 || len > 3) continue;
        bool all_digit = true;
        for (size_t k = 0; k < len; k++) {
            if (f[k] < '0' || f[k] > '9') { all_digit = false; break; }
        }
        if (!all_digit) continue;

        // Next field (MNC) must also be decimal.
        const char *mnc = fields[i + 1];
        bool mnc_digit = mnc[0] != '\0';
        for (const char *p = mnc; *p; p++) {
            if (*p < '0' || *p > '9') { mnc_digit = false; break; }
        }
        if (!mnc_digit) continue;

        out->mcc = (uint32_t)atoi(f);
        out->mnc = (uint32_t)atoi(mnc);
        out->lac = (uint32_t)strtol(fields[i + 2], NULL, 16);
        out->cid = (uint32_t)strtol(fields[i + 3], NULL, 16);
        out->valid = true;
        return true;
    }
    return false;
}
