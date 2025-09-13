package org.firstinspires.ftc.ftccommon.external;

import android.content.Context;
import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * {@link OnCreate} provides an easy way to perform initialization when the robot controller
 * activity is created.
 *
 * <p>Place an OnCreate annotation on a public static method in your code, and that method will be
 * automatically called from FtcRobotControllerActivity.onCreate. The method must take a
 * {@link Context} as its only parameter.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
@Documented
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface OnCreate {
}
