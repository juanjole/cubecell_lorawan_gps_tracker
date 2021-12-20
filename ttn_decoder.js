function decodeUplink(input) {

    var buffer = new ArrayBuffer(input.bytes.length);
    var dataView = new DataView(buffer);
    input.bytes.forEach(function (value, index) {
        dataView.setUint8(index, value);
    });

    var latitude = dataView.getFloat32(0, true);
    var longitude = dataView.getFloat32(4, true);
    var altitude = dataView.getFloat32(8, true);
    var hdop = dataView.getFloat32(12, true);
    var sats = dataView.getUint32(16, true);
    var time = dataView.getUint32(20, true);
    var date = dataView.getUint32(24, true);

    var battery = dataView.getUint16(28, true);


    return {
        data: {
            bytes: input.bytes,
            latitude: latitude,
            longitude: longitude,
            altitude: altitude,
            hdop: hdop,
            sats: sats,
            time: time,
            date: date,
            battery: battery
        },
        warnings: [],
        errors: []
    };
}