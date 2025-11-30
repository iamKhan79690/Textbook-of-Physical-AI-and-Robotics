"""
Database initialization script for creating tables and indexes.
Run this script to initialize the Neon Postgres database schema.
"""
import asyncio
import asyncpg
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import settings


async def init_database():
    """Initialize database schema by running schema.sql."""
    print("Connecting to database...")
    
    # Validate DATABASE_URL is set
    if not settings.database_url:
        print("\n✗ Error: DATABASE_URL not set in .env file")
        print("Please create a .env file in the backend/ directory with:")
        print("DATABASE_URL=postgresql://user:password@host.neon.tech/database?sslmode=require")
        sys.exit(1)
    
    # Show masked URL for debugging (hide password)
    db_url_display = settings.database_url
    if '@' in db_url_display:
        # Mask password in display
        parts = db_url_display.split('@')
        if ':' in parts[0]:
            user_pass = parts[0].split(':')
            if len(user_pass) >= 3:  # postgresql://user:pass
                db_url_display = f"{user_pass[0]}:{user_pass[1]}:***@{parts[1]}"
    
    print(f"Database URL: {db_url_display[:50]}...")  # Show first 50 chars

    try:
        # Use connect with explicit dsn parameter (same as test script)
        print("  Establishing connection (timeout: 30s)...")
        conn = await asyncio.wait_for(
            asyncpg.connect(dsn=settings.database_url),
            timeout=30.0
        )
        print("✓ Connected to database")

        # Read schema SQL file
        schema_path = Path(__file__).parent / "schema.sql"
        with open(schema_path, 'r', encoding='utf-8') as f:
            schema_sql = f.read()

        print("Executing schema SQL...")
        await conn.execute(schema_sql)
        print("✓ Database schema created successfully")

        # Verify tables were created
        tables = await conn.fetch("""
            SELECT tablename FROM pg_tables
            WHERE schemaname = 'public'
            ORDER BY tablename;
        """)

        print("\nCreated tables:")
        for table in tables:
            print(f"  - {table['tablename']}")

        await conn.close()
        print("\n✓ Database initialization complete")

    except Exception as e:
        print(f"\n✗ Error initializing database: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(init_database())
