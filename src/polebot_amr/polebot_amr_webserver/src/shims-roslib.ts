const ROSLIB = (window as any).ROSLIB;

if (!ROSLIB) {
  throw new Error('ROSLIB not found on window');
}

export default ROSLIB;
