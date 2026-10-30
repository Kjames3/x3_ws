const fs = require('fs'), vm = require('vm'), assert = require('assert');
const source = fs.readFileSync('src/web/main.js','utf8');
const scope = {TextDecoder, Float32Array, DataView, Uint8Array};
vm.createContext(scope);
vm.runInContext(source.slice(source.indexOf('class CDRReader'),source.indexOf('function parseLaserScanCDR'))+source.slice(source.indexOf('function parseSurroundingsCDR'),source.indexOf('/**\n * Parse a Foxglove binary MessageData'))+';this.parse=parseSurroundingsCDR;',scope);
const cdr=fs.readFileSync(process.argv[2]);const b=new Uint8Array(13+cdr.length);b.set(cdr,13);
const xyz=scope.parse(b.buffer,13);assert(xyz.length>0 && xyz.length<=18000);assert([...xyz].every(Number.isFinite));
assert.throws(()=>scope.parse(b.buffer.slice(0,100),13));
console.log('Browser decoder: real ROS CDR with Foxglove offset and truncated input checks passed');
if (process.argv[3]) {
    vm.runInContext('this.parsePose=parseSurroundingsPoseCDR;',scope);
    const poseCdr=fs.readFileSync(process.argv[3]), poseFrame=new Uint8Array(13+poseCdr.length);
    poseFrame.set(poseCdr,13);
    const pose=scope.parsePose(poseFrame.buffer,13);
    assert(Math.abs(pose.stamp-xyz.stamp)<1e-8);
    assert(Math.abs(pose.x-.1)<1e-6);assert(Math.abs(pose.yaw-.1)<1e-6);
    console.log('Matching cloud/PoseStamped CDR and yaw decoding passed');
}
