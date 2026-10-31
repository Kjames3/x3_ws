const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync('src/web/main.js','utf8');
const scope={TextDecoder,Float32Array,DataView,Uint8Array};vm.createContext(scope);
vm.runInContext(source.slice(source.indexOf('class CDRReader'),source.indexOf('function parseLaserScanCDR'))+source.slice(source.indexOf('function parseSurroundingsCDR'),source.indexOf('/**\n * Parse a Foxglove binary MessageData'))+';this.parse=parseSurroundingsCDR;',scope);
const cdr=fs.readFileSync(process.argv[2]),frame=new Uint8Array(13+cdr.length);frame.set(cdr,13);
const cloud=scope.parse(frame.buffer,13,true);
assert(cloud.length>0 && cloud.length<=18000 && cloud.colors.length===cloud.length);
assert([...cloud].every(Number.isFinite));assert([...cloud.colors].every(c=>c>=0&&c<=1));
assert.throws(()=>scope.parse(frame.buffer.slice(0,100),13,true));
assert.throws(()=>scope.parse(frame.buffer,13,false));
if(process.argv[3]==='fixture') {
 assert.deepStrictEqual([...cloud],[1,2,3,4,5,6]);
 assert.deepStrictEqual([...cloud.colors],[1,0,0,0,1,0]);
}
console.log(JSON.stringify({points:cloud.length/3,stamp:cloud.stamp,valid:true}));
