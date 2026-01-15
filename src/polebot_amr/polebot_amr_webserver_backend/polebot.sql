-- --------------------------------------------------------
-- Host:                         127.0.0.1
-- Server version:               8.0.30 - MySQL Community Server - GPL
-- Server OS:                    Win64
-- HeidiSQL Version:             12.1.0.6537
-- --------------------------------------------------------

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET NAMES utf8 */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;


-- Dumping database structure for polebot
CREATE DATABASE IF NOT EXISTS `polebot` /*!40100 DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci */ /*!80016 DEFAULT ENCRYPTION='N' */;
USE `polebot`;

-- Dumping structure for table polebot.categories
CREATE TABLE IF NOT EXISTS `categories` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(255) NOT NULL,
  `description` text,
  `icon` varchar(50) DEFAULT 'fas fa-folder',
  `color` varchar(20) DEFAULT '#3b82f6',
  `created_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  KEY `idx_categories_name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

-- Dumping structure for event polebot.cleanup_unused_goal_sets
DELIMITER //
CREATE EVENT `cleanup_unused_goal_sets` ON SCHEDULE EVERY 1 MONTH STARTS '2025-12-12 10:59:55' ON COMPLETION NOT PRESERVE ENABLE DO BEGIN
    DELETE FROM goal_sets 
    WHERE id NOT IN (
        SELECT DISTINCT g.goal_set_id 
        FROM goals g
        JOIN mission_goals mg ON g.id = mg.goal_id
    )
    AND created_at < DATE_SUB(NOW(), INTERVAL 6 MONTH);
END//
DELIMITER ;

-- Dumping structure for function polebot.CountMissionsByCategory
DELIMITER //
CREATE FUNCTION `CountMissionsByCategory`(category_id INT) RETURNS int
    READS SQL DATA
BEGIN
    DECLARE mission_count INT;
    SELECT COUNT(*) INTO mission_count FROM missions WHERE category_id = category_id;
    RETURN mission_count;
END//
DELIMITER ;

-- Dumping structure for procedure polebot.GetMissionWithTriggers
DELIMITER //
CREATE PROCEDURE `GetMissionWithTriggers`(IN mission_id INT)
BEGIN
    -- Get mission basic info
    SELECT 
        m.id,
        m.name,
        m.description,
        m.category_id,
        c.name as category_name,
        c.icon as category_icon,
        c.color as category_color,
        m.created_at,
        m.last_executed
    FROM missions m
    LEFT JOIN categories c ON m.category_id = c.id
    WHERE m.id = mission_id;
    
    -- Get mission goals with trigger details
    SELECT 
        mg.id,
        mg.sequence_number,
        mg.next_goal_trigger,
        mg.wait_time,
        mg.sensor_type,
        mg.sensor_condition,
        mg.timeout,
        mg.retry_count,
        mg.on_failure,
        g.position_x,
        g.position_y,
        g.orientation_z,
        g.orientation_w,
        g.tolerance_xy,
        g.tolerance_yaw,
        g.created_at as goal_created_at,
        gs.name as goal_set_name,
        gs.id as goal_set_id,
        mp.name as map_name
    FROM mission_goals mg
    JOIN goals g ON mg.goal_id = g.id
    LEFT JOIN goal_sets gs ON g.goal_set_id = gs.id
    LEFT JOIN maps mp ON gs.map_id = mp.id
    WHERE mg.mission_id = mission_id
    ORDER BY mg.sequence_number;
END//
DELIMITER ;

-- Dumping structure for table polebot.maps
CREATE TABLE IF NOT EXISTS `maps` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(255) NOT NULL,
  `resolution` decimal(10,5) DEFAULT '0.05000',
  `origin_x` decimal(10,5) DEFAULT '0.00000',
  `origin_y` decimal(10,5) DEFAULT '0.00000',
  `width` int DEFAULT '0',
  `height` int DEFAULT '0',
  `created_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `name` (`name`),
  KEY `idx_maps_name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

-- Dumping structure for table polebot.goal_sets
CREATE TABLE IF NOT EXISTS `goal_sets` (
  `id` int NOT NULL AUTO_INCREMENT,
  `map_id` int NOT NULL,
  `name` varchar(255) NOT NULL,
  `description` text,
  `total_goals` int DEFAULT '0',
  `created_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `unique_set_name` (`name`),
  KEY `idx_goal_sets_map` (`map_id`),
  KEY `idx_goal_sets_name` (`name`),
  CONSTRAINT `goal_sets_ibfk_1` FOREIGN KEY (`map_id`) REFERENCES `maps` (`id`) ON DELETE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

-- Dumping structure for table polebot.goals
CREATE TABLE IF NOT EXISTS `goals` (
  `id` int NOT NULL AUTO_INCREMENT,
  `goal_set_id` int NOT NULL,
  `sequence_number` int NOT NULL,
  `position_x` decimal(10,5) NOT NULL,
  `position_y` decimal(10,5) NOT NULL,
  `orientation_z` decimal(10,5) DEFAULT '0.00000',
  `orientation_w` decimal(10,5) DEFAULT '1.00000',
  `tolerance_xy` decimal(10,5) DEFAULT '0.30000',
  `tolerance_yaw` decimal(10,5) DEFAULT '0.50000',
  `created_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `unique_goal_sequence` (`goal_set_id`,`sequence_number`),
  KEY `idx_goals_set` (`goal_set_id`),
  KEY `idx_goals_sequence` (`sequence_number`),
  CONSTRAINT `goals_ibfk_1` FOREIGN KEY (`goal_set_id`) REFERENCES `goal_sets` (`id`) ON DELETE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=18 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

-- Dumping structure for view polebot.goal_sets_summary
-- Creating temporary table to overcome VIEW dependency errors
CREATE TABLE `goal_sets_summary` (
	`id` INT(10) NOT NULL,
	`name` VARCHAR(255) NOT NULL COLLATE 'utf8mb4_unicode_ci',
	`description` TEXT NULL COLLATE 'utf8mb4_unicode_ci',
	`map_name` VARCHAR(255) NOT NULL COLLATE 'utf8mb4_unicode_ci',
	`total_goals` INT(10) NULL,
	`saved_goals` BIGINT(19) NOT NULL,
	`created_at` TIMESTAMP NULL
) ENGINE=MyISAM;

-- Dumping structure for table polebot.missions
CREATE TABLE IF NOT EXISTS `missions` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(255) NOT NULL,
  `description` text,
  `category_id` int DEFAULT '1',
  `last_executed` timestamp NULL DEFAULT NULL,
  `created_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  KEY `idx_missions_name` (`name`),
  KEY `idx_missions_category` (`category_id`),
  CONSTRAINT `missions_ibfk_1` FOREIGN KEY (`category_id`) REFERENCES `categories` (`id`) ON DELETE SET DEFAULT
) ENGINE=InnoDB AUTO_INCREMENT=18 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

-- Dumping structure for view polebot.missions_with_triggers_summary
-- Creating temporary table to overcome VIEW dependency errors
CREATE TABLE `missions_with_triggers_summary` (
	`id` INT(10) NOT NULL,
	`name` VARCHAR(255) NOT NULL COLLATE 'utf8mb4_unicode_ci',
	`description` TEXT NULL COLLATE 'utf8mb4_unicode_ci',
	`category_id` INT(10) NULL,
	`category_name` VARCHAR(255) NULL COLLATE 'utf8mb4_unicode_ci',
	`category_icon` VARCHAR(50) NULL COLLATE 'utf8mb4_unicode_ci',
	`category_color` VARCHAR(20) NULL COLLATE 'utf8mb4_unicode_ci',
	`goal_count` BIGINT(19) NOT NULL,
	`triggers_count` DECIMAL(23,0) NULL,
	`created_at` TIMESTAMP NULL,
	`last_executed` TIMESTAMP NULL
) ENGINE=MyISAM;

-- Dumping structure for table polebot.mission_goals
CREATE TABLE IF NOT EXISTS `mission_goals` (
  `id` int NOT NULL AUTO_INCREMENT,
  `mission_id` int NOT NULL,
  `goal_id` int NOT NULL,
  `sequence_number` int NOT NULL,
  `next_goal_trigger` varchar(20) DEFAULT 'auto',
  `wait_time` int DEFAULT '5',
  `sensor_type` varchar(50) DEFAULT '',
  `sensor_condition` text,
  `timeout` int DEFAULT '60',
  `retry_count` int DEFAULT '0',
  `on_failure` varchar(20) DEFAULT 'skip',
  `created_at` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `unique_mission_goal` (`mission_id`,`goal_id`),
  KEY `idx_mission_goals_mission` (`mission_id`),
  KEY `idx_mission_goals_goal` (`goal_id`),
  KEY `idx_mission_goals_sequence` (`sequence_number`),
  KEY `idx_mission_goals_trigger` (`next_goal_trigger`),
  CONSTRAINT `mission_goals_ibfk_1` FOREIGN KEY (`mission_id`) REFERENCES `missions` (`id`) ON DELETE CASCADE,
  CONSTRAINT `mission_goals_ibfk_2` FOREIGN KEY (`goal_id`) REFERENCES `goals` (`id`) ON DELETE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=15 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

-- Dumping structure for trigger polebot.update_mission_timestamp
SET @OLDTMP_SQL_MODE=@@SQL_MODE, SQL_MODE='ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION';
DELIMITER //
CREATE TRIGGER `update_mission_timestamp` AFTER INSERT ON `mission_goals` FOR EACH ROW BEGIN
    UPDATE missions 
    SET updated_at = CURRENT_TIMESTAMP 
    WHERE id = NEW.mission_id;
END//
DELIMITER ;
SET SQL_MODE=@OLDTMP_SQL_MODE;

-- Dumping structure for trigger polebot.update_mission_timestamp_on_delete
SET @OLDTMP_SQL_MODE=@@SQL_MODE, SQL_MODE='ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION';
DELIMITER //
CREATE TRIGGER `update_mission_timestamp_on_delete` AFTER DELETE ON `mission_goals` FOR EACH ROW BEGIN
    UPDATE missions 
    SET updated_at = CURRENT_TIMESTAMP 
    WHERE id = OLD.mission_id;
END//
DELIMITER ;
SET SQL_MODE=@OLDTMP_SQL_MODE;

-- Dumping structure for trigger polebot.update_mission_timestamp_on_update
SET @OLDTMP_SQL_MODE=@@SQL_MODE, SQL_MODE='ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION';
DELIMITER //
CREATE TRIGGER `update_mission_timestamp_on_update` AFTER UPDATE ON `mission_goals` FOR EACH ROW BEGIN
    UPDATE missions 
    SET updated_at = CURRENT_TIMESTAMP 
    WHERE id = NEW.mission_id;
END//
DELIMITER ;
SET SQL_MODE=@OLDTMP_SQL_MODE;

-- Dumping structure for view polebot.goal_sets_summary
-- Removing temporary table and create final VIEW structure
DROP TABLE IF EXISTS `goal_sets_summary`;
CREATE ALGORITHM=UNDEFINED SQL SECURITY DEFINER VIEW `goal_sets_summary` AS select `gs`.`id` AS `id`,`gs`.`name` AS `name`,`gs`.`description` AS `description`,`m`.`name` AS `map_name`,`gs`.`total_goals` AS `total_goals`,count(`g`.`id`) AS `saved_goals`,`gs`.`created_at` AS `created_at` from ((`goal_sets` `gs` join `maps` `m` on((`gs`.`map_id` = `m`.`id`))) left join `goals` `g` on((`gs`.`id` = `g`.`goal_set_id`))) group by `gs`.`id`,`gs`.`name`,`gs`.`description`,`m`.`name`,`gs`.`total_goals`,`gs`.`created_at`;

-- Dumping structure for view polebot.missions_with_triggers_summary
-- Removing temporary table and create final VIEW structure
DROP TABLE IF EXISTS `missions_with_triggers_summary`;
CREATE ALGORITHM=UNDEFINED SQL SECURITY DEFINER VIEW `missions_with_triggers_summary` AS select `m`.`id` AS `id`,`m`.`name` AS `name`,`m`.`description` AS `description`,`m`.`category_id` AS `category_id`,`c`.`name` AS `category_name`,`c`.`icon` AS `category_icon`,`c`.`color` AS `category_color`,count(`mg`.`id`) AS `goal_count`,sum((case when (`mg`.`next_goal_trigger` <> 'auto') then 1 else 0 end)) AS `triggers_count`,`m`.`created_at` AS `created_at`,`m`.`last_executed` AS `last_executed` from ((`missions` `m` left join `categories` `c` on((`m`.`category_id` = `c`.`id`))) left join `mission_goals` `mg` on((`m`.`id` = `mg`.`mission_id`))) group by `m`.`id`,`m`.`name`,`m`.`description`,`m`.`category_id`,`c`.`name`,`c`.`icon`,`c`.`color`,`m`.`created_at`,`m`.`last_executed`;

/*!40103 SET TIME_ZONE=IFNULL(@OLD_TIME_ZONE, 'system') */;
/*!40101 SET SQL_MODE=IFNULL(@OLD_SQL_MODE, '') */;
/*!40014 SET FOREIGN_KEY_CHECKS=IFNULL(@OLD_FOREIGN_KEY_CHECKS, 1) */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40111 SET SQL_NOTES=IFNULL(@OLD_SQL_NOTES, 1) */;
