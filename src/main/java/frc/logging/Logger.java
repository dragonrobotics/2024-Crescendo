package frc.logging;

import java.lang.reflect.Field;

public class Logger {

    public static void AutoLog(Object root){
        AutoLog(root, "");
    };
    
    public static void AutoLog(Object root, String path){
        System.out.println(root.getClass().getSimpleName());
        for (Field field : root.getClass().getDeclaredFields()) {
            System.out.println(field.getName());
        }
    }
}
