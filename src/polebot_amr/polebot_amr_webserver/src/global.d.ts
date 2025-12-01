export {};

declare global {
  const ROS2D: any;
  const createjs: any;

  interface Window {
    ROS2D: any;
    ROSLIB: any;
    createjs: any;
  }
}
