const assert = require('node:assert/strict');
const {analyze} = require('../../src/web/surroundings-geometry.js');
const near = (a,b) => assert(Math.abs(a-b)<1e-6, `${a} != ${b}`);
let r = analyze([], .4);
assert.equal(r.front, Infinity); assert.equal(r.overhead, Infinity); assert.equal(r.gap, null);
r=analyze([1,0,0, NaN,1,1],.4); assert.equal(r.front,Infinity); // floor isn't a front obstacle
r=analyze([.51,0,.2],.4);near(r.front,.34);assert.equal(r.cells[0].risk,2);
r=analyze([.01,.01,.2],.4);assert(r.overlap);near(r.front,0);
r=analyze([.51,0,.61],.4);near(r.overhead,.2);assert.equal(r.front,Infinity);
r=analyze([.51,.31,.2,.51,-.31,.2],.4);near(r.gap.width,.6);near(r.gap.spare,.38);
r=analyze([.51,.31,.2,.91,-.31,.2],.4);assert.equal(r.gap,null); // sides at different depths aren't a doorway
r=analyze([0,.31,.2,0,-.31,.2],.4);near(r.left,.19);near(r.right,.19);
r=analyze([1,1,1,1.01,1.01,1.01],.4);assert.equal(r.cells.length,1);
r=analyze([.51,.31,.2,.51,-.31,.2,.51,0,.2],.4);assert.equal(r.gap,null); // blocked gap is not an opening
console.log('Surroundings geometry checks passed');
