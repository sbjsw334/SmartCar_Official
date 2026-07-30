#include "bsp_oled.h"

#include "ti_msp_dl_config.h"

#define BSP_OLED_ADDRESS          (0x3CU)
#define BSP_OLED_WIDTH            (128U)
#define BSP_OLED_PAGES            (8U)
#define BSP_OLED_TEXT_ROWS        (8U)
#define BSP_OLED_TEXT_COLS        (21U)
#define BSP_OLED_CHAR_WIDTH       (6U)
#define BSP_OLED_TRANSFER_TIMEOUT (100000U)
#define BSP_OLED_DATA_CHUNK       (16U)

static uint8_t s_ready = 0U;
static char s_lineCache[BSP_OLED_TEXT_ROWS][BSP_OLED_TEXT_COLS + 1U];
static uint8_t s_lineValidMask = 0U;

static uint8_t _WritePacket(const uint8_t *pData, uint16_t length);
static uint8_t _WriteCommand(uint8_t command);
static uint8_t _WriteData(const uint8_t *pData, uint8_t length);
static uint8_t _SetCursor(uint8_t page, uint8_t column);
static void _ShowChar(uint8_t row, uint8_t col, char character);
static const uint8_t *_GetGlyph(char character);
static void _ShowLineIfChanged(uint8_t row, const char *line);
static void _ClearLine(char *line);
static void _AppendChar(char *line, uint8_t *pIndex, char character);
static void _AppendText(char *line, uint8_t *pIndex, const char *text);
static void _AppendUnsigned(char *line, uint8_t *pIndex, uint32_t value,
    uint8_t digits);
static void _AppendSigned3(char *line, uint8_t *pIndex, int16_t value);
static char _HexDigit(uint8_t value);

void BspOled_Init(void)
{
    static const uint8_t initCommands[] = {
        0xAEU, 0x20U, 0x00U, 0xB0U, 0xC8U, 0x00U, 0x10U, 0x40U,
        0x81U, 0x7FU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
        0x00U, 0xD5U, 0x80U, 0xD9U, 0xF1U, 0xDAU, 0x12U, 0xDBU,
        0x40U, 0x8DU, 0x14U, 0xAFU,
    };
    uint8_t index;

    delay_cycles(320000U);
    s_ready = 1U;

    for (index = 0U; index < sizeof(initCommands); index++) {
        if (_WriteCommand(initCommands[index]) == 0U) {
            s_ready = 0U;
            return;
        }
    }

    BspOled_Clear();
}

void BspOled_Clear(void)
{
    uint8_t zeros[BSP_OLED_DATA_CHUNK] = {0U};
    uint8_t page;
    uint8_t column;

    if (s_ready == 0U) {
        return;
    }

    for (page = 0U; page < BSP_OLED_PAGES; page++) {
        if (_SetCursor(page, 0U) == 0U) {
            return;
        }
        for (column = 0U; column < BSP_OLED_WIDTH;
             column += BSP_OLED_DATA_CHUNK) {
            if (_WriteData(zeros, sizeof(zeros)) == 0U) {
                return;
            }
        }
    }

    s_lineValidMask = 0U;
}

void BspOled_ShowText(uint8_t row, uint8_t col, const char *text)
{
    if ((s_ready == 0U) || (text == 0) || (row >= BSP_OLED_PAGES)) {
        return;
    }

    while ((*text != '\0') && (col < BSP_OLED_TEXT_COLS)) {
        _ShowChar(row, col, *text);
        col++;
        text++;
    }
}

void BspOled_ShowStatus(const BspOledStatusView_t *pView)
{
    static const char *const fatherNames[] = {
        "STOP", "RUN", "DONE", "FAULT",
    };
    char line[BSP_OLED_TEXT_COLS + 1U];
    uint8_t index;
    uint32_t elapsedSeconds;
    uint16_t elapsedHundredths;
    const char *fatherName = "FAULT";

    if ((s_ready == 0U) || (pView == 0)) {
        return;
    }

    if (pView->fatherState <
        (uint8_t)(sizeof(fatherNames) / sizeof(fatherNames[0]))) {
        fatherName = fatherNames[pView->fatherState];
    }

    _ClearLine(line);
    index = 0U;
    _AppendChar(line, &index, 'H');
    _AppendUnsigned(line, &index, (uint16_t)(pView->mode + 2U), 1U);
    _AppendText(line, &index, "  ");
    _AppendText(line, &index, fatherName);
    _ShowLineIfChanged(0U, line);

    _ClearLine(line);
    index = 0U;
    elapsedSeconds = pView->elapsedMs / 1000U;
    if (elapsedSeconds > 9999U) {
        elapsedSeconds = 9999U;
    }
    _AppendText(line, &index, "TIME ");
    _AppendUnsigned(line, &index, elapsedSeconds, 4U);
    _AppendChar(line, &index, '.');
    elapsedHundredths = (uint16_t)((pView->elapsedMs % 1000U) / 10U);
    _AppendUnsigned(line, &index, elapsedHundredths, 2U);
    _AppendText(line, &index, " S");
    _ShowLineIfChanged(1U, line);

    _ClearLine(line);
    index = 0U;
    _AppendText(line, &index, "ENC ");
    _AppendUnsigned(line, &index, pView->routePulses, 6U);
    _ShowLineIfChanged(2U, line);

    _ClearLine(line);
    index = 0U;
    _AppendText(line, &index, "TARGET ");
    _AppendSigned3(line, &index, pView->ballTargetMm);
    _AppendText(line, &index, " MM");
    _ShowLineIfChanged(3U, line);

    _ClearLine(line);
    index = 0U;
    _AppendText(line, &index, "BALL ");
    _AppendSigned3(line, &index, pView->ballOffsetMm);
    _AppendText(line, &index, " MM V");
    _AppendUnsigned(line, &index, pView->ballValid, 1U);
    _ShowLineIfChanged(4U, line);

    _ClearLine(line);
    index = 0U;
    _AppendText(line, &index, "GRAY ");
    _AppendChar(line, &index, _HexDigit((uint8_t)(pView->gray >> 4)));
    _AppendChar(line, &index, _HexDigit(pView->gray));
    _ShowLineIfChanged(5U, line);
}

static uint8_t _WritePacket(const uint8_t *pData, uint16_t length)
{
    uint16_t written;
    uint32_t timeout = BSP_OLED_TRANSFER_TIMEOUT;

    if ((pData == 0) || (length == 0U)) {
        return 0U;
    }

    while (((DL_I2C_getControllerStatus(I2C_OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        return 0U;
    }

    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    written = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, pData, length);
    DL_I2C_startControllerTransfer(I2C_OLED_INST, BSP_OLED_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);

    /* Required settling time after START for MSPM0 I2C_ERR_13. */
    delay_cycles(100U);

    timeout = BSP_OLED_TRANSFER_TIMEOUT;
    while (((DL_I2C_getControllerStatus(I2C_OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) && (timeout > 0U)) {
        if (written < length) {
            written += DL_I2C_fillControllerTXFIFO(I2C_OLED_INST,
                &pData[written], (uint16_t)(length - written));
        }
        timeout--;
    }

    if ((timeout == 0U) ||
        ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
          DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) ||
        (written != length)) {
        DL_I2C_resetControllerTransfer(I2C_OLED_INST);
        DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
        return 0U;
    }

    return 1U;
}

static uint8_t _WriteCommand(uint8_t command)
{
    uint8_t packet[2] = {0x00U, command};

    return _WritePacket(packet, sizeof(packet));
}

static uint8_t _WriteData(const uint8_t *pData, uint8_t length)
{
    uint8_t packet[BSP_OLED_DATA_CHUNK + 1U];
    uint8_t index;

    if ((pData == 0) || (length == 0U) ||
        (length > BSP_OLED_DATA_CHUNK)) {
        return 0U;
    }

    packet[0] = 0x40U;
    for (index = 0U; index < length; index++) {
        packet[index + 1U] = pData[index];
    }

    return _WritePacket(packet, (uint16_t)(length + 1U));
}

static uint8_t _SetCursor(uint8_t page, uint8_t column)
{
    uint8_t packet[4] = {
        0x00U,
        (uint8_t)(0xB0U + page),
        (uint8_t)(column & 0x0FU),
        (uint8_t)(0x10U | (column >> 4)),
    };

    return _WritePacket(packet, sizeof(packet));
}

static void _ShowChar(uint8_t row, uint8_t col, char character)
{
    uint8_t data[BSP_OLED_CHAR_WIDTH];
    const uint8_t *glyph = _GetGlyph(character);
    uint8_t index;

    if (_SetCursor(row, (uint8_t)(col * BSP_OLED_CHAR_WIDTH)) == 0U) {
        return;
    }
    for (index = 0U; index < 5U; index++) {
        data[index] = glyph[index];
    }
    data[5] = 0U;
    (void)_WriteData(data, sizeof(data));
}

static const uint8_t *_GetGlyph(char character)
{
    static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
    static const uint8_t plus[5] = {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
    static const uint8_t minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
    static const uint8_t dot[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
    static const uint8_t digits[10][5] = {
        {0x3EU,0x51U,0x49U,0x45U,0x3EU}, {0x00U,0x42U,0x7FU,0x40U,0x00U},
        {0x42U,0x61U,0x51U,0x49U,0x46U}, {0x21U,0x41U,0x45U,0x4BU,0x31U},
        {0x18U,0x14U,0x12U,0x7FU,0x10U}, {0x27U,0x45U,0x45U,0x45U,0x39U},
        {0x3CU,0x4AU,0x49U,0x49U,0x30U}, {0x01U,0x71U,0x09U,0x05U,0x03U},
        {0x36U,0x49U,0x49U,0x49U,0x36U}, {0x06U,0x49U,0x49U,0x29U,0x1EU},
    };
    static const uint8_t letters[26][5] = {
        {0x7EU,0x11U,0x11U,0x11U,0x7EU}, {0x7FU,0x49U,0x49U,0x49U,0x36U},
        {0x3EU,0x41U,0x41U,0x41U,0x22U}, {0x7FU,0x41U,0x41U,0x22U,0x1CU},
        {0x7FU,0x49U,0x49U,0x49U,0x41U}, {0x7FU,0x09U,0x09U,0x09U,0x01U},
        {0x3EU,0x41U,0x49U,0x49U,0x7AU}, {0x7FU,0x08U,0x08U,0x08U,0x7FU},
        {0x00U,0x41U,0x7FU,0x41U,0x00U}, {0x20U,0x40U,0x41U,0x3FU,0x01U},
        {0x7FU,0x08U,0x14U,0x22U,0x41U}, {0x7FU,0x40U,0x40U,0x40U,0x40U},
        {0x7FU,0x02U,0x0CU,0x02U,0x7FU}, {0x7FU,0x04U,0x08U,0x10U,0x7FU},
        {0x3EU,0x41U,0x41U,0x41U,0x3EU}, {0x7FU,0x09U,0x09U,0x09U,0x06U},
        {0x3EU,0x41U,0x51U,0x21U,0x5EU}, {0x7FU,0x09U,0x19U,0x29U,0x46U},
        {0x46U,0x49U,0x49U,0x49U,0x31U}, {0x01U,0x01U,0x7FU,0x01U,0x01U},
        {0x3FU,0x40U,0x40U,0x40U,0x3FU}, {0x1FU,0x20U,0x40U,0x20U,0x1FU},
        {0x7FU,0x20U,0x18U,0x20U,0x7FU}, {0x63U,0x14U,0x08U,0x14U,0x63U},
        {0x07U,0x08U,0x70U,0x08U,0x07U}, {0x61U,0x51U,0x49U,0x45U,0x43U},
    };

    if ((character >= '0') && (character <= '9')) {
        return digits[(uint8_t)(character - '0')];
    }
    if ((character >= 'A') && (character <= 'Z')) {
        return letters[(uint8_t)(character - 'A')];
    }
    if (character == '+') {
        return plus;
    }
    if (character == '-') {
        return minus;
    }
    if (character == '.') {
        return dot;
    }
    return blank;
}

static void _ShowLineIfChanged(uint8_t row, const char *line)
{
    uint8_t pixels[BSP_OLED_TEXT_COLS * BSP_OLED_CHAR_WIDTH];
    const uint8_t *glyph;
    uint8_t firstChanged = 0U;
    uint8_t lastChanged = (BSP_OLED_TEXT_COLS - 1U);
    uint8_t charIndex;
    uint8_t glyphIndex;
    uint8_t pixelCount;
    uint8_t sent;
    uint8_t chunk;
    uint8_t index;

    if ((row >= BSP_OLED_TEXT_ROWS) || (line == 0)) {
        return;
    }

    if ((s_lineValidMask & (uint8_t)(1U << row)) != 0U) {
        while ((firstChanged < BSP_OLED_TEXT_COLS) &&
               (s_lineCache[row][firstChanged] == line[firstChanged])) {
            firstChanged++;
        }
        if (firstChanged == BSP_OLED_TEXT_COLS) {
            return;
        }
        while ((lastChanged > firstChanged) &&
               (s_lineCache[row][lastChanged] == line[lastChanged])) {
            lastChanged--;
        }
    }

    pixelCount = 0U;
    for (charIndex = firstChanged; charIndex <= lastChanged; charIndex++) {
        glyph = _GetGlyph(line[charIndex]);
        for (glyphIndex = 0U; glyphIndex < 5U; glyphIndex++) {
            pixels[pixelCount++] = glyph[glyphIndex];
        }
        pixels[pixelCount++] = 0U;
    }

    if (_SetCursor(row,
            (uint8_t)(firstChanged * BSP_OLED_CHAR_WIDTH)) == 0U) {
        return;
    }
    sent = 0U;
    while (sent < pixelCount) {
        chunk = (uint8_t)(pixelCount - sent);
        if (chunk > BSP_OLED_DATA_CHUNK) {
            chunk = BSP_OLED_DATA_CHUNK;
        }
        if (_WriteData(&pixels[sent], chunk) == 0U) {
            return;
        }
        sent = (uint8_t)(sent + chunk);
    }

    for (index = 0U; index < BSP_OLED_TEXT_COLS; index++) {
        s_lineCache[row][index] = line[index];
    }
    s_lineCache[row][BSP_OLED_TEXT_COLS] = '\0';
    s_lineValidMask |= (uint8_t)(1U << row);
}

static void _ClearLine(char *line)
{
    uint8_t index;

    for (index = 0U; index < BSP_OLED_TEXT_COLS; index++) {
        line[index] = ' ';
    }
    line[BSP_OLED_TEXT_COLS] = '\0';
}

static void _AppendChar(char *line, uint8_t *pIndex, char character)
{
    if (*pIndex < BSP_OLED_TEXT_COLS) {
        line[*pIndex] = character;
        (*pIndex)++;
    }
}

static void _AppendText(char *line, uint8_t *pIndex, const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        _AppendChar(line, pIndex, *text);
        text++;
    }
}

static void _AppendUnsigned(char *line, uint8_t *pIndex, uint32_t value,
    uint8_t digits)
{
    uint32_t divisor = 1U;
    uint8_t digit;

    for (digit = 1U; digit < digits; digit++) {
        divisor *= 10U;
    }
    while (digits > 0U) {
        _AppendChar(line, pIndex, (char)('0' + ((value / divisor) % 10U)));
        divisor /= 10U;
        digits--;
    }
}

static void _AppendSigned3(char *line, uint8_t *pIndex, int16_t value)
{
    int32_t magnitude = value;

    if (magnitude < 0) {
        _AppendChar(line, pIndex, '-');
        magnitude = -magnitude;
    } else {
        _AppendChar(line, pIndex, '+');
    }
    if (magnitude > 999) {
        magnitude = 999;
    }
    _AppendUnsigned(line, pIndex, (uint16_t)magnitude, 3U);
}

static char _HexDigit(uint8_t value)
{
    value &= 0x0FU;
    return (value < 10U) ? (char)('0' + value) : (char)('A' + value - 10U);
}
