/*
A minimal Web Bluetooth connection example
created 6 Aug 2018
by Tom Igoe
*/
var myDevice;
var myService = 0xffb0;        // fill in a service you're looking for here
var myCharacteristic = 0x2AB3;   // fill in a characteristic from the service here

function connect(){
  navigator.bluetooth.requestDevice({
    // filters: [myFilters]       // you can't use filters and acceptAllDevices together
    optionalServices: [myService],
    acceptAllDevices: true
  })
  .then(function(device) {
    // save the device returned so you can disconnect later:
    myDevice = device;
    // connect to the device once you find it:
    return device.gatt.connect();
  })
  .then(function(server) {
    // get the primary service:
    return server.getPrimaryService(myService);
  })
  .then(function(service) {
    // get the  characteristic:
    return service.getCharacteristics();
  })
  .then(function(characteristics) {
    // subscribe to the characteristic:
    for (c in characteristics) {
      characteristics[c].startNotifications()
      .then(subscribeToChanges);
      //characteristics[c].addEventListener('characteristicvaluechanged', handleData)
    }
  })
  .catch(function(error) {
    // catch any errors:
    console.error('Connection failed!', error);
  });
}

// subscribe to changes from the meter:
function subscribeToChanges(characteristic) {
  characteristic.oncharacteristicvaluechanged = handleData;
}

// handle incoming data:
function handleData(event) {
  // get the data buffer from the meter:
  var buf = new Uint8Array(event.target.value);

  console.log(event.target.value);



  //var x = event.target.value.getFloat32(0,true); 
  //var y = event.target.value.getFloat32(4,true);
  //var z = event.target.value.getFloat32(8,true);

  //console.log(x);
  //console.log(y);
  //console.log(z);

}

// disconnect function:
function disconnect() {
  if (myDevice) {
    // disconnect:
    myDevice.gatt.disconnect();
  }
}