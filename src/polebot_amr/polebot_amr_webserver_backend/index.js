const express = require('express');
const cors = require('cors');
const mysql = require('mysql2/promise');
require('dotenv').config();

const app = express();
const PORT = process.env.PORT || 3000;

// Middleware
app.use(cors());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Database connection pool
const pool = mysql.createPool({
    host: 'localhost',
    user: 'root',
    password: 'amr2025',
    database: 'polebot',
    waitForConnections: true,
    connectionLimit: 10,
    queueLimit: 0,
    enableKeepAlive: true,
    keepAliveInitialDelay: 0
});

// Test database connection
async function testDatabaseConnection() {
    try {
        const connection = await pool.getConnection();
        console.log('✅ Database connected successfully');
        connection.release();
        return true;
    } catch (error) {
        console.error('❌ Database connection failed:', error.message);
        return false;
    }
}

// API Routes

// 1. Health Check
app.get('/api/health', async (req, res) => {
    const dbConnected = await testDatabaseConnection();

    res.json({
        status: 'ok',
        service: 'nav2-goals-api',
        version: '1.0.0',
        database: dbConnected ? 'connected' : 'disconnected',
        timestamp: new Date().toISOString()
    });
});

// 2. Create/Save Goal Set
app.post('/api/goals/save', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        await connection.beginTransaction();

        const {
            mapName,
            setName,
            goals,
            mapInfo,
            description
        } = req.body;

        if (!mapName || !setName || !goals || !Array.isArray(goals)) {
            return res.status(400).json({
                error: 'Missing required fields: mapName, setName, goals'
            });
        }

        if (goals.length === 0) {
            return res.status(400).json({
                error: 'Goals array cannot be empty'
            });
        }

        // 1. Create or get map
        const [mapResult] = await connection.execute(
            `INSERT INTO maps (name, resolution, origin_x, origin_y, width, height) 
             VALUES (?, ?, ?, ?, ?, ?)
             ON DUPLICATE KEY UPDATE 
                resolution = VALUES(resolution),
                origin_x = VALUES(origin_x),
                origin_y = VALUES(origin_y),
                width = VALUES(width),
                height = VALUES(height),
                updated_at = CURRENT_TIMESTAMP`,
            [
                mapName,
                mapInfo?.resolution || 0.05,
                mapInfo?.origin?.x || 0,
                mapInfo?.origin?.y || 0,
                mapInfo?.width || 0,
                mapInfo?.height || 0
            ]
        );

        const mapId = mapResult.insertId || (
            await connection.execute('SELECT id FROM maps WHERE name = ?', [mapName])
        )[0][0].id;

        // 2. Create goal set
        const [setResult] = await connection.execute(
            `INSERT INTO goal_sets (map_id, name, description, total_goals) 
             VALUES (?, ?, ?, ?)
             ON DUPLICATE KEY UPDATE
                description = VALUES(description),
                total_goals = VALUES(total_goals),
                created_at = CURRENT_TIMESTAMP`,
            [mapId, setName, description || '', goals.length]
        );

        const goalSetId = setResult.insertId || (
            await connection.execute('SELECT id FROM goal_sets WHERE name = ?', [setName])
        )[0][0].id;

        // 3. Delete existing goals for this set (if updating)
        await connection.execute(
            'DELETE FROM goals WHERE goal_set_id = ?',
            [goalSetId]
        );

        // 4. Insert new goals
        const goalValues = goals.map((goal, index) => [
            goalSetId,
            index + 1, // sequence number
            goal.x,
            goal.y,
            goal.orientation_z || 0.0,
            goal.orientation_w || 1.0,
            goal.tolerance_xy || 0.3,
            goal.tolerance_yaw || 0.5
        ]);

        if (goalValues.length > 0) {
            await connection.query(
                `INSERT INTO goals 
                 (goal_set_id, sequence_number, position_x, position_y, 
                  orientation_z, orientation_w, tolerance_xy, tolerance_yaw) 
                 VALUES ?`,
                [goalValues]
            );
        }

        // 5. Update total goals count
        await connection.execute(
            'UPDATE goal_sets SET total_goals = ? WHERE id = ?',
            [goals.length, goalSetId]
        );

        await connection.commit();

        res.status(201).json({
            success: true,
            message: 'Goal set saved successfully',
            data: {
                goalSetId,
                setName,
                mapName,
                totalGoals: goals.length,
                goals: goals.map((g, i) => ({
                    sequence: i + 1,
                    x: g.x,
                    y: g.y
                }))
            }
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error saving goal set:', error);
        res.status(500).json({
            error: 'Failed to save goal set',
            details: error.message
        });
    } finally {
        connection.release();
    }
});

// 3. Get all goal sets
app.get('/api/goals/sets', async (req, res) => {
    try {
        const [goalSets] = await pool.execute(`
            SELECT 
                gs.id,
                gs.name as set_name,
                gs.description,
                m.name as map_name,
                gs.total_goals,
                COUNT(g.id) as saved_goals,
                gs.created_at
            FROM goal_sets gs
            JOIN maps m ON gs.map_id = m.id
            LEFT JOIN goals g ON gs.id = g.goal_set_id
            GROUP BY gs.id, gs.name, gs.description, m.name, gs.total_goals, gs.created_at
            ORDER BY gs.created_at DESC
        `);

        res.json({
            success: true,
            count: goalSets.length,
            goalSets
        });

    } catch (error) {
        console.error('Error fetching goal sets:', error);
        res.status(500).json({ error: error.message });
    }
});

// 4. Get goal set by name
app.get('/api/goals/set/:setName', async (req, res) => {
    try {
        const { setName } = req.params;

        // Get goal set info
        const [setRows] = await pool.execute(`
            SELECT 
                gs.*,
                m.name as map_name,
                m.resolution,
                m.origin_x,
                m.origin_y,
                m.width,
                m.height
            FROM goal_sets gs
            JOIN maps m ON gs.map_id = m.id
            WHERE gs.name = ?
        `, [setName]);

        if (setRows.length === 0) {
            return res.status(404).json({ error: 'Goal set not found' });
        }

        // Get goals for this set
        const [goals] = await pool.execute(`
            SELECT 
                id,
                sequence_number,
                position_x as x,
                position_y as y,
                orientation_z,
                orientation_w,
                tolerance_xy,
                tolerance_yaw,
                created_at
            FROM goals 
            WHERE goal_set_id = ?
            ORDER BY sequence_number
        `, [setRows[0].id]);

        res.json({
            success: true,
            goalSet: setRows[0],
            goals,
            totalGoals: goals.length
        });

    } catch (error) {
        console.error('Error fetching goal set:', error);
        res.status(500).json({ error: error.message });
    }
});

// 5. Get goal set by ID
app.get('/api/goals/set/id/:id', async (req, res) => {
    try {
        const { id } = req.params;

        // Get goal set info
        const [setRows] = await pool.execute(`
            SELECT 
                gs.*,
                m.name as map_name,
                m.resolution,
                m.origin_x,
                m.origin_y,
                m.width,
                m.height
            FROM goal_sets gs
            JOIN maps m ON gs.map_id = m.id
            WHERE gs.id = ?
        `, [id]);

        if (setRows.length === 0) {
            return res.status(404).json({ error: 'Goal set not found' });
        }

        // Get goals for this set
        const [goals] = await pool.execute(`
            SELECT 
                id,
                sequence_number,
                position_x as x,
                position_y as y,
                orientation_z,
                orientation_w,
                tolerance_xy,
                tolerance_yaw,
                created_at
            FROM goals 
            WHERE goal_set_id = ?
            ORDER BY sequence_number
        `, [id]);

        res.json({
            success: true,
            goalSet: setRows[0],
            goals,
            totalGoals: goals.length
        });

    } catch (error) {
        console.error('Error fetching goal set:', error);
        res.status(500).json({ error: error.message });
    }
});

// 6. Update goal set (replace goals)
app.put('/api/goals/set/:setName', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        await connection.beginTransaction();

        const { setName } = req.params;
        const { goals, description } = req.body;

        if (!goals || !Array.isArray(goals)) {
            return res.status(400).json({
                error: 'Missing required field: goals'
            });
        }

        // Get existing goal set
        const [setRows] = await connection.execute(
            'SELECT id FROM goal_sets WHERE name = ?',
            [setName]
        );

        if (setRows.length === 0) {
            await connection.rollback();
            return res.status(404).json({ error: 'Goal set not found' });
        }

        const goalSetId = setRows[0].id;

        // Update description if provided
        if (description !== undefined) {
            await connection.execute(
                'UPDATE goal_sets SET description = ? WHERE id = ?',
                [description, goalSetId]
            );
        }

        // Delete existing goals
        await connection.execute(
            'DELETE FROM goals WHERE goal_set_id = ?',
            [goalSetId]
        );

        // Insert new goals
        if (goals.length > 0) {
            const goalValues = goals.map((goal, index) => [
                goalSetId,
                index + 1,
                goal.x,
                goal.y,
                goal.orientation_z || 0.0,
                goal.orientation_w || 1.0,
                goal.tolerance_xy || 0.3,
                goal.tolerance_yaw || 0.5
            ]);

            await connection.query(
                `INSERT INTO goals 
                 (goal_set_id, sequence_number, position_x, position_y, 
                  orientation_z, orientation_w, tolerance_xy, tolerance_yaw) 
                 VALUES ?`,
                [goalValues]
            );
        }

        // Update total goals count
        await connection.execute(
            'UPDATE goal_sets SET total_goals = ? WHERE id = ?',
            [goals.length, goalSetId]
        );

        await connection.commit();

        res.json({
            success: true,
            message: 'Goal set updated successfully',
            data: {
                goalSetId,
                setName,
                totalGoals: goals.length
            }
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error updating goal set:', error);
        res.status(500).json({
            error: 'Failed to update goal set',
            details: error.message
        });
    } finally {
        connection.release();
    }
});

// 7. Delete goal set
app.delete('/api/goals/set/:setName', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        const { setName } = req.params;

        await connection.beginTransaction();

        // Get goal set info before deletion
        const [setRows] = await connection.execute(
            'SELECT id FROM goal_sets WHERE name = ?',
            [setName]
        );

        if (setRows.length === 0) {
            await connection.rollback();
            return res.status(404).json({ error: 'Goal set not found' });
        }

        // Delete goal set (cascade will delete goals)
        const [result] = await connection.execute(
            'DELETE FROM goal_sets WHERE name = ?',
            [setName]
        );

        await connection.commit();

        res.json({
            success: true,
            message: 'Goal set deleted successfully',
            setName
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error deleting goal set:', error);
        res.status(500).json({ error: error.message });
    } finally {
        connection.release();
    }
});

// 8. Add single goal to existing set
app.post('/api/goals/set/:setName/add', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        await connection.beginTransaction();

        const { setName } = req.params;
        const { x, y, orientation_z, orientation_w } = req.body;

        if (x === undefined || y === undefined) {
            return res.status(400).json({
                error: 'Missing required fields: x, y'
            });
        }

        // Get goal set
        const [setRows] = await connection.execute(
            'SELECT id, total_goals FROM goal_sets WHERE name = ?',
            [setName]
        );

        if (setRows.length === 0) {
            await connection.rollback();
            return res.status(404).json({ error: 'Goal set not found' });
        }

        const goalSetId = setRows[0].id;
        const nextSequence = setRows[0].total_goals + 1;

        // Insert new goal
        await connection.execute(
            `INSERT INTO goals 
             (goal_set_id, sequence_number, position_x, position_y, 
              orientation_z, orientation_w, tolerance_xy, tolerance_yaw) 
             VALUES (?, ?, ?, ?, ?, ?, ?, ?)`,
            [
                goalSetId,
                nextSequence,
                x,
                y,
                orientation_z || 0.0,
                orientation_w || 1.0,
                0.3,
                0.5
            ]
        );

        // Update total goals count
        await connection.execute(
            'UPDATE goal_sets SET total_goals = total_goals + 1 WHERE id = ?',
            [goalSetId]
        );

        await connection.commit();

        res.status(201).json({
            success: true,
            message: 'Goal added successfully',
            data: {
                goalSetId,
                setName,
                sequence: nextSequence,
                goal: { x, y }
            }
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error adding goal:', error);
        res.status(500).json({
            error: 'Failed to add goal',
            details: error.message
        });
    } finally {
        connection.release();
    }
});

// 9. Remove goal from set by sequence
app.delete('/api/goals/set/:setName/:sequence', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        const { setName, sequence } = req.params;

        await connection.beginTransaction();

        // Get goal set
        const [setRows] = await connection.execute(
            'SELECT id FROM goal_sets WHERE name = ?',
            [setName]
        );

        if (setRows.length === 0) {
            await connection.rollback();
            return res.status(404).json({ error: 'Goal set not found' });
        }

        const goalSetId = setRows[0].id;

        // Delete the goal
        const [result] = await connection.execute(
            'DELETE FROM goals WHERE goal_set_id = ? AND sequence_number = ?',
            [goalSetId, sequence]
        );

        if (result.affectedRows === 0) {
            await connection.rollback();
            return res.status(404).json({ error: 'Goal not found' });
        }

        // Update sequence numbers of remaining goals
        await connection.execute(
            `UPDATE goals 
             SET sequence_number = sequence_number - 1 
             WHERE goal_set_id = ? AND sequence_number > ?`,
            [goalSetId, sequence]
        );

        // Update total goals count
        await connection.execute(
            'UPDATE goal_sets SET total_goals = total_goals - 1 WHERE id = ?',
            [goalSetId]
        );

        await connection.commit();

        res.json({
            success: true,
            message: 'Goal removed successfully',
            data: {
                goalSetId,
                setName,
                removedSequence: sequence
            }
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error removing goal:', error);
        res.status(500).json({ error: error.message });
    } finally {
        connection.release();
    }
});

// 10. Update specific goal
app.put('/api/goals/set/:setName/:sequence', async (req, res) => {
    try {
        const { setName, sequence } = req.params;
        const { x, y, orientation_z, orientation_w } = req.body;

        if (x === undefined || y === undefined) {
            return res.status(400).json({ error: 'Missing x or y coordinates' });
        }

        // Get goal set id first
        const [setRows] = await pool.execute(
            'SELECT id FROM goal_sets WHERE name = ?',
            [setName]
        );

        if (setRows.length === 0) {
            return res.status(404).json({ error: 'Goal set not found' });
        }

        const goalSetId = setRows[0].id;

        const [result] = await pool.execute(
            `UPDATE goals 
             SET position_x = ?, position_y = ?, 
                 orientation_z = ?, orientation_w = ?,
                 created_at = CURRENT_TIMESTAMP
             WHERE goal_set_id = ? AND sequence_number = ?`,
            [
                x,
                y,
                orientation_z || 0.0,
                orientation_w || 1.0,
                goalSetId,
                sequence
            ]
        );

        if (result.affectedRows === 0) {
            return res.status(404).json({
                error: 'Goal not found'
            });
        }

        res.json({
            success: true,
            message: 'Goal updated successfully',
            data: {
                goalSetId,
                setName,
                sequence,
                goal: { x, y }
            }
        });

    } catch (error) {
        console.error('Error updating goal:', error);
        res.status(500).json({ error: error.message });
    }
});

// 11. Search goal sets
app.get('/api/goals/search', async (req, res) => {
    try {
        const { name, mapName, minGoals, maxGoals } = req.query;

        let query = `
            SELECT 
                gs.id,
                gs.name as set_name,
                gs.description,
                m.name as map_name,
                gs.total_goals,
                gs.created_at
            FROM goal_sets gs
            JOIN maps m ON gs.map_id = m.id
            WHERE 1=1
        `;

        const params = [];

        if (name) {
            query += ' AND gs.name LIKE ?';
            params.push(`%${name}%`);
        }

        if (mapName) {
            query += ' AND m.name LIKE ?';
            params.push(`%${mapName}%`);
        }

        if (minGoals) {
            query += ' AND gs.total_goals >= ?';
            params.push(minGoals);
        }

        if (maxGoals) {
            query += ' AND gs.total_goals <= ?';
            params.push(maxGoals);
        }

        query += ' ORDER BY gs.created_at DESC';

        const [goalSets] = await pool.execute(query, params);

        res.json({
            success: true,
            count: goalSets.length,
            goalSets
        });

    } catch (error) {
        console.error('Error searching goal sets:', error);
        res.status(500).json({ error: error.message });
    }
});

// Categories API
app.post('/api/categories', async (req, res) => {
    try {
        const { name, description, icon, color } = req.body;

        if (!name) {
            return res.status(400).json({ error: 'Category name is required' });
        }

        const [result] = await pool.execute(
            'INSERT INTO categories (name, description, icon, color) VALUES (?, ?, ?, ?)',
            [name, description || '', icon || 'fas fa-folder', color || '#3b82f6']
        );

        res.status(201).json({
            success: true,
            category: {
                id: result.insertId,
                name,
                description,
                icon,
                color
            }
        });

    } catch (error) {
        console.error('Error creating category:', error);
        res.status(500).json({ error: error.message });
    }
});

app.get('/api/categories', async (req, res) => {
    try {
        const [categories] = await pool.execute(`
            SELECT c.*, COUNT(m.id) as mission_count
            FROM categories c
            LEFT JOIN missions m ON c.id = m.category_id
            GROUP BY c.id
            ORDER BY c.name
        `);

        res.json({
            success: true,
            categories
        });

    } catch (error) {
        console.error('Error fetching categories:', error);
        res.status(500).json({ error: error.message });
    }
});

app.put('/api/categories/:id', async (req, res) => {
    try {
        const { id } = req.params;
        const { name, description, icon, color } = req.body;

        const [result] = await pool.execute(
            'UPDATE categories SET name = ?, description = ?, icon = ?, color = ? WHERE id = ?',
            [name, description || '', icon || 'fas fa-folder', color || '#3b82f6', id]
        );

        if (result.affectedRows === 0) {
            return res.status(404).json({ error: 'Category not found' });
        }

        res.json({
            success: true,
            message: 'Category updated successfully'
        });

    } catch (error) {
        console.error('Error updating category:', error);
        res.status(500).json({ error: error.message });
    }
});

app.delete('/api/categories/:id', async (req, res) => {
    try {
        const { id } = req.params;

        // First, move missions to default category
        await pool.execute(
            'UPDATE missions SET category_id = 1 WHERE category_id = ?',
            [id]
        );

        const [result] = await pool.execute(
            'DELETE FROM categories WHERE id = ?',
            [id]
        );

        if (result.affectedRows === 0) {
            return res.status(404).json({ error: 'Category not found' });
        }

        res.json({
            success: true,
            message: 'Category deleted successfully'
        });

    } catch (error) {
        console.error('Error deleting category:', error);
        res.status(500).json({ error: error.message });
    }
});

// GET all missions with their goals - PERBAIKAN QUERY
// GET all missions with their goals - PERBAIKAN QUERY
app.get('/api/missions', async (req, res) => {
    try {
        const [missions] = await pool.execute(`
            SELECT 
                m.id,
                m.name,
                m.description,
                m.category_id,
                m.created_at,
                m.last_executed,
                c.name as category_name,
                c.icon as category_icon,
                c.color as category_color,
                COUNT(mg.id) as goal_count
            FROM missions m
            LEFT JOIN categories c ON m.category_id = c.id
            LEFT JOIN mission_goals mg ON m.id = mg.mission_id
            GROUP BY m.id, m.name, m.description, m.category_id, 
                     m.created_at, m.last_executed, c.name, c.icon, c.color
            ORDER BY m.created_at DESC
        `);

        // For each mission, fetch its goals
        for (const mission of missions) {
            const [goals] = await pool.execute(`
                SELECT 
                    g.id,
                    g.position_x,
                    g.position_y,
                    g.orientation_z,
                    g.orientation_w,
                    g.tolerance_xy,
                    g.tolerance_yaw,
                    g.created_at,
                    mg.sequence_number,
                    gs.name as goal_set_name,
                    mp.name as map_name
                FROM mission_goals mg
                JOIN goals g ON mg.goal_id = g.id
                JOIN goal_sets gs ON g.goal_set_id = gs.id
                JOIN maps mp ON gs.map_id = mp.id
                WHERE mg.mission_id = ?
                ORDER BY mg.sequence_number
            `, [mission.id]);

            mission.goals = goals;
        }

        res.json({
            success: true,
            missions,
            count: missions.length
        });

    } catch (error) {
        console.error('Error fetching missions:', error);
        res.status(500).json({ error: error.message });
    }
});

// GET single mission by ID - PERBAIKAN QUERY
app.get('/api/missions/:id', async (req, res) => {
    try {
        const { id } = req.params;

        const [missions] = await pool.execute(`
            SELECT 
                m.id,
                m.name,
                m.description,
                m.category_id,
                m.created_at,
                m.last_executed,
                c.name as category_name,
                c.icon as category_icon,
                c.color as category_color
            FROM missions m
            LEFT JOIN categories c ON m.category_id = c.id
            WHERE m.id = ?
        `, [id]);

        if (missions.length === 0) {
            return res.status(404).json({ error: 'Mission not found' });
        }

        const mission = missions[0];

        // Get mission goals
        const [goals] = await pool.execute(`
            SELECT 
                g.id,
                g.position_x,
                g.position_y,
                g.orientation_z,
                g.orientation_w,
                g.tolerance_xy,
                g.tolerance_yaw,
                g.created_at,
                mg.sequence_number,
                gs.name as goal_set_name,
                mp.name as map_name
            FROM mission_goals mg
            JOIN goals g ON mg.goal_id = g.id
            JOIN goal_sets gs ON g.goal_set_id = gs.id
            JOIN maps mp ON gs.map_id = mp.id
            WHERE mg.mission_id = ?
            ORDER BY mg.sequence_number
        `, [id]);

        mission.goals = goals;
        mission.goal_count = goals.length;

        res.json({
            success: true,
            mission
        });

    } catch (error) {
        console.error('Error fetching mission:', error);
        res.status(500).json({ error: error.message });
    }
});

// POST create new mission - PERBAIKAN UNTUK TABLE missions YANG TIDAK ADA KOLOM status
app.post('/api/missions', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        await connection.beginTransaction();

        const { name, description, category_id, goals } = req.body;

        if (!name || !category_id) {
            return res.status(400).json({
                error: 'Mission name and category are required'
            });
        }

        const [result] = await connection.execute(
            'INSERT INTO missions (name, description, category_id) VALUES (?, ?, ?)',
            [name, description || '', category_id]
        );

        const missionId = result.insertId;

        // Save mission goals
        if (goals && Array.isArray(goals) && goals.length > 0) {
            const goalValues = goals.map((goal, index) => [
                missionId,
                goal.id,
                index + 1
            ]);

            await connection.query(
                'INSERT INTO mission_goals (mission_id, goal_id, sequence_number) VALUES ?',
                [goalValues]
            );
        }

        await connection.commit();

        res.status(201).json({
            success: true,
            message: 'Mission created successfully',
            mission: {
                id: missionId,
                name,
                description,
                category_id
            }
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error creating mission:', error);
        res.status(500).json({ error: error.message });
    } finally {
        connection.release();
    }
});

// PUT update mission - PERBAIKAN UNTUK TABLE missions YANG TIDAK ADA KOLOM status
app.put('/api/missions/:id', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        await connection.beginTransaction();

        const { id } = req.params;
        const { name, description, category_id, goals } = req.body;

        // Update mission details
        const [result] = await connection.execute(
            'UPDATE missions SET name = ?, description = ?, category_id = ? WHERE id = ?',
            [name, description || '', category_id, id]
        );

        if (result.affectedRows === 0) {
            await connection.rollback();
            return res.status(404).json({ error: 'Mission not found' });
        }

        // Remove existing mission goals
        await connection.execute(
            'DELETE FROM mission_goals WHERE mission_id = ?',
            [id]
        );

        // Add new mission goals if provided
        if (goals && Array.isArray(goals) && goals.length > 0) {
            const goalValues = goals.map((goal, index) => [
                id,
                goal.id,
                index + 1
            ]);

            await connection.query(
                'INSERT INTO mission_goals (mission_id, goal_id, sequence_number) VALUES ?',
                [goalValues]
            );
        }

        await connection.commit();

        res.json({
            success: true,
            message: 'Mission updated successfully'
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error updating mission:', error);
        res.status(500).json({ error: error.message });
    } finally {
        connection.release();
    }
});

// DELETE mission
app.delete('/api/missions/:id', async (req, res) => {
    try {
        const { id } = req.params;

        const [result] = await pool.execute(
            'DELETE FROM missions WHERE id = ?',
            [id]
        );

        if (result.affectedRows === 0) {
            return res.status(404).json({ error: 'Mission not found' });
        }

        res.json({
            success: true,
            message: 'Mission deleted successfully'
        });

    } catch (error) {
        console.error('Error deleting mission:', error);
        res.status(500).json({ error: error.message });
    }
});

// POST save mission with triggers
app.post('/api/missions/with-triggers', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        await connection.beginTransaction();

        const { name, description, category_id, goals } = req.body;

        if (!name || !category_id) {
            return res.status(400).json({
                error: 'Mission name and category are required'
            });
        }

        if (!goals || !Array.isArray(goals) || goals.length === 0) {
            return res.status(400).json({
                error: 'Mission must have at least one goal'
            });
        }

        // Create mission
        const [result] = await connection.execute(
            'INSERT INTO missions (name, description, category_id) VALUES (?, ?, ?)',
            [name, description || '', category_id]
        );

        const missionId = result.insertId;

        // Save mission goals with triggers
        const goalValues = [];
        for (let i = 0; i < goals.length; i++) {
            const goal = goals[i];
            
            // First, save goal to goals table if it doesn't exist
            let goalId = goal.id;
            
            if (!goalId) {
                // Insert new goal
                const [goalResult] = await connection.execute(
                    `INSERT INTO goals 
                     (goal_set_id, sequence_number, position_x, position_y, 
                      orientation_z, orientation_w, tolerance_xy, tolerance_yaw) 
                     VALUES (?, ?, ?, ?, ?, ?, ?, ?)`,
                    [
                        goal.goal_set_id || 1, // default goal set
                        goal.sequence_number || (i + 1),
                        goal.position_x || 0,
                        goal.position_y || 0,
                        goal.orientation_z || 0.0,
                        goal.orientation_w || 1.0,
                        goal.tolerance_xy || 0.3,
                        goal.tolerance_yaw || 0.5
                    ]
                );
                goalId = goalResult.insertId;
            }

            // Save to mission_goals with trigger data
            goalValues.push([
                missionId,
                goalId,
                i + 1, // sequence_number
                goal.next_goal_trigger || 'auto',
                goal.wait_time || 5,
                goal.sensor_type || '',
                goal.sensor_condition || '',
                goal.timeout || 60,
                goal.retry_count || 0,
                goal.on_failure || 'skip'
            ]);
        }

        // Insert mission goals with triggers
        if (goalValues.length > 0) {
            await connection.query(
                `INSERT INTO mission_goals 
                 (mission_id, goal_id, sequence_number, next_goal_trigger, 
                  wait_time, sensor_type, sensor_condition, timeout, 
                  retry_count, on_failure) 
                 VALUES ?`,
                [goalValues]
            );
        }

        await connection.commit();

        // Fetch the complete mission data
        const [missions] = await connection.execute(`
            SELECT 
                m.*,
                c.name as category_name,
                c.icon as category_icon
            FROM missions m
            LEFT JOIN categories c ON m.category_id = c.id
            WHERE m.id = ?
        `, [missionId]);

        const mission = missions[0];

        // Fetch goals with trigger data
        const [missionGoals] = await connection.execute(`
            SELECT 
                mg.*,
                g.position_x,
                g.position_y,
                g.orientation_z,
                g.orientation_w,
                g.tolerance_xy,
                g.tolerance_yaw,
                gs.name as goal_set_name,
                mp.name as map_name
            FROM mission_goals mg
            JOIN goals g ON mg.goal_id = g.id
            LEFT JOIN goal_sets gs ON g.goal_set_id = gs.id
            LEFT JOIN maps mp ON gs.map_id = mp.id
            WHERE mg.mission_id = ?
            ORDER BY mg.sequence_number
        `, [missionId]);

        mission.goals = missionGoals;
        mission.goal_count = missionGoals.length;

        connection.release();

        res.status(201).json({
            success: true,
            message: 'Mission saved with triggers successfully',
            mission
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error saving mission with triggers:', error);
        res.status(500).json({ 
            error: 'Failed to save mission with triggers',
            details: error.message 
        });
    }
});

// PUT update mission with triggers
app.put('/api/missions/with-triggers/:id', async (req, res) => {
    const connection = await pool.getConnection();

    try {
        await connection.beginTransaction();

        const { id } = req.params;
        const { name, description, category_id, goals } = req.body;

        // Check if mission exists
        const [existingMission] = await connection.execute(
            'SELECT id FROM missions WHERE id = ?',
            [id]
        );

        if (existingMission.length === 0) {
            await connection.rollback();
            return res.status(404).json({ error: 'Mission not found' });
        }

        // Update mission details
        await connection.execute(
            'UPDATE missions SET name = ?, description = ?, category_id = ? WHERE id = ?',
            [name, description || '', category_id, id]
        );

        // Remove existing mission goals
        await connection.execute(
            'DELETE FROM mission_goals WHERE mission_id = ?',
            [id]
        );

        // Save new mission goals with triggers
        if (goals && Array.isArray(goals) && goals.length > 0) {
            const goalValues = [];
            
            for (let i = 0; i < goals.length; i++) {
                const goal = goals[i];
                let goalId = goal.id;

                if (!goalId) {
                    // Insert new goal if it doesn't exist
                    const [goalResult] = await connection.execute(
                        `INSERT INTO goals 
                         (goal_set_id, sequence_number, position_x, position_y, 
                          orientation_z, orientation_w, tolerance_xy, tolerance_yaw) 
                         VALUES (?, ?, ?, ?, ?, ?, ?, ?)`,
                        [
                            goal.goal_set_id || 1,
                            goal.sequence_number || (i + 1),
                            goal.position_x || 0,
                            goal.position_y || 0,
                            goal.orientation_z || 0.0,
                            goal.orientation_w || 1.0,
                            goal.tolerance_xy || 0.3,
                            goal.tolerance_yaw || 0.5
                        ]
                    );
                    goalId = goalResult.insertId;
                }

                goalValues.push([
                    id,
                    goalId,
                    i + 1,
                    goal.next_goal_trigger || 'auto',
                    goal.wait_time || 5,
                    goal.sensor_type || '',
                    goal.sensor_condition || '',
                    goal.timeout || 60,
                    goal.retry_count || 0,
                    goal.on_failure || 'skip'
                ]);
            }

            if (goalValues.length > 0) {
                await connection.query(
                    `INSERT INTO mission_goals 
                     (mission_id, goal_id, sequence_number, next_goal_trigger, 
                      wait_time, sensor_type, sensor_condition, timeout, 
                      retry_count, on_failure) 
                     VALUES ?`,
                    [goalValues]
                );
            }
        }

        await connection.commit();

        // Fetch updated mission data
        const [missions] = await connection.execute(`
            SELECT 
                m.*,
                c.name as category_name,
                c.icon as category_icon
            FROM missions m
            LEFT JOIN categories c ON m.category_id = c.id
            WHERE m.id = ?
        `, [id]);

        const mission = missions[0];

        // Fetch goals with trigger data
        const [missionGoals] = await connection.execute(`
            SELECT 
                mg.*,
                g.position_x,
                g.position_y,
                g.orientation_z,
                g.orientation_w,
                g.tolerance_xy,
                g.tolerance_yaw,
                gs.name as goal_set_name,
                mp.name as map_name
            FROM mission_goals mg
            JOIN goals g ON mg.goal_id = g.id
            LEFT JOIN goal_sets gs ON g.goal_set_id = gs.id
            LEFT JOIN maps mp ON gs.map_id = mp.id
            WHERE mg.mission_id = ?
            ORDER BY mg.sequence_number
        `, [id]);

        mission.goals = missionGoals;
        mission.goal_count = missionGoals.length;

        connection.release();

        res.json({
            success: true,
            message: 'Mission updated with triggers successfully',
            mission
        });

    } catch (error) {
        await connection.rollback();
        console.error('Error updating mission with triggers:', error);
        res.status(500).json({ 
            error: 'Failed to update mission with triggers',
            details: error.message 
        });
    }
});

// GET mission with triggers by ID
app.get('/api/missions/with-triggers/:id', async (req, res) => {
    try {
        const { id } = req.params;

        const [missions] = await pool.execute(`
            SELECT 
                m.id,
                m.name,
                m.description,
                m.category_id,
                m.created_at,
                m.last_executed,
                c.name as category_name,
                c.icon as category_icon,
                c.color as category_color
            FROM missions m
            LEFT JOIN categories c ON m.category_id = c.id
            WHERE m.id = ?
        `, [id]);

        if (missions.length === 0) {
            return res.status(404).json({ error: 'Mission not found' });
        }

        const mission = missions[0];

        // Get mission goals with triggers
        const [goals] = await pool.execute(`
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
                g.created_at,
                gs.name as goal_set_name,
                gs.id as goal_set_id,
                mp.name as map_name
            FROM mission_goals mg
            JOIN goals g ON mg.goal_id = g.id
            LEFT JOIN goal_sets gs ON g.goal_set_id = gs.id
            LEFT JOIN maps mp ON gs.map_id = mp.id
            WHERE mg.mission_id = ?
            ORDER BY mg.sequence_number
        `, [id]);

        mission.goals = goals;
        mission.goal_count = goals.length;

        res.json({
            success: true,
            mission
        });

    } catch (error) {
        console.error('Error fetching mission with triggers:', error);
        res.status(500).json({ error: error.message });
    }
});

// Update mission last_executed timestamp
app.patch('/api/missions/:id/execute', async (req, res) => {
    try {
        const { id } = req.params;

        const [result] = await pool.execute(
            'UPDATE missions SET last_executed = CURRENT_TIMESTAMP WHERE id = ?',
            [id]
        );

        if (result.affectedRows === 0) {
            return res.status(404).json({ error: 'Mission not found' });
        }

        res.json({
            success: true,
            message: 'Mission execution timestamp updated successfully'
        });

    } catch (error) {
        console.error('Error updating mission execution timestamp:', error);
        res.status(500).json({ error: error.message });
    }
});

// Error handling middleware
app.use((err, req, res, next) => {
    console.error(err.stack);
    res.status(500).json({
        error: 'Internal server error',
        message: err.message
    });
});

// 404 handler
app.use((req, res) => {
    res.status(404).json({
        error: 'Endpoint not found',
        path: req.path
    });
});

// Start server
app.listen(PORT, async () => {
    console.log(`🚀 Goals API Server running on http://localhost:${PORT}`);

    // Test database connection
    const dbConnected = await testDatabaseConnection();
    if (!dbConnected) {
        console.warn('⚠️  Server started without database connection');
    }
});