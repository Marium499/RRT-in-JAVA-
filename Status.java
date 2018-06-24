

public enum Status {
	
		ADVANCE{

			@Override
			public int getValue() {
				
				return 0;
			}
			
			
		},
		REACHED{

			@Override
			public int getValue() {
				
				return 1;
			}
			
			
		}, 
		TRAPPED{

			@Override
			public int getValue() {
				
				return 2;
			}
			
			
		},;
		  public abstract int getValue() ;
	}
