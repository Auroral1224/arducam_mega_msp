void framWriteBuffer(uint8_t* fram, uint8_t* buff, uint32_t length)
{
    while (length > 0)
    {
        *(fram)  = *(buff);
        fram++;
        buff+=2;
        length -= 2;
    }
    return;
}

void uartTestWriteBuffer(uint8_t* buff, uint32_t length)
{
    while (length > 0)
    {
        arducamUartWrite(*(buff));
        buff+=2;
        length -= 2;
    }
    return;
}
void cameraGetPicture96x96YUV(ArducamCamera *camera)
{
    uint8_t *fram_addr = YUVarr;
    uint8_t buff[READ_IMAGE_LENGTH] = { 0 };
    uint8_t rtLength = 0;

    while (camera->receivedLength)
    {
        rtLength = readBuff(camera, buff, READ_IMAGE_LENGTH);
        framWriteBuffer(fram_addr, buff, rtLength);
        // uartTestWriteBuffer(buff, rtLength);
        fram_addr += rtLength / 2;
    }
}
