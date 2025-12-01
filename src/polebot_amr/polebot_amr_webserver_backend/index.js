// server.js
const express = require('express');
const mysql = require('mysql2');
const cors = require('cors');
const app = express();

app.use(cors());
app.use(express.json());

// Koneksi MySQL
const db = mysql.createConnection({
  host: 'localhost',
  user: 'admin',
  password: 'passwordku',
  database: 'polebot_amr'
});

db.connect(err => {
  if (err) {
    console.error('Database connection failed:', err);
    return;
  }
  console.log('Connected to MySQL database');
  
  const createTables = `
    CREATE TABLE IF NOT EXISTS missions (
      id INT AUTO_INCREMENT PRIMARY KEY,
      name VARCHAR(255) NOT NULL,
      type ENUM('goal', 'no_go_zone') NOT NULL,
      coordinates JSON NOT NULL,
      created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
      updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
    )
  `;
  
  db.execute(createTables);
});

// Routes
// Get all missions
app.get('/api/missions', (req, res) => {
  const query = 'SELECT * FROM missions ORDER BY created_at DESC';
  db.execute(query, (err, results) => {
    if (err) {
      res.status(500).json({ error: err.message });
      return;
    }
    res.json(results);
  });
});

// Add new mission
app.post('/api/missions', (req, res) => {
  const { name, type, coordinates } = req.body;
  const query = 'INSERT INTO missions (name, type, coordinates) VALUES (?, ?, ?)';
  
  db.execute(query, [name, type, JSON.stringify(coordinates)], (err, results) => {
    if (err) {
      res.status(500).json({ error: err.message });
      return;
    }
    res.json({ id: results.insertId, message: 'Mission added successfully' });
  });
});

// Update mission
app.put('/api/missions/:id', (req, res) => {
  const { id } = req.params;
  const { name, coordinates } = req.body;
  const query = 'UPDATE missions SET name = ?, coordinates = ? WHERE id = ?';
  
  db.execute(query, [name, JSON.stringify(coordinates), id], (err, results) => {
    if (err) {
      res.status(500).json({ error: err.message });
      return;
    }
    res.json({ message: 'Mission updated successfully' });
  });
});

// Delete mission
app.delete('/api/missions/:id', (req, res) => {
  const { id } = req.params;
  const query = 'DELETE FROM missions WHERE id = ?';
  
  db.execute(query, [id], (err, results) => {
    if (err) {
      res.status(500).json({ error: err.message });
      return;
    }
    res.json({ message: 'Mission deleted successfully' });
  });
});

const PORT = 3001;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});